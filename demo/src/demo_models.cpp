// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

#pragma once

#include <string>
#include <stdint.h>
#include <vector>
#include <iostream>

#include <discrete_mirage.h>

#include "example_implementations.c"

#include "math_utils.cpp"
#include "helper_utils.cpp"

const int MODEL_GEOMETRY_OCTREE = 1;
const int MODEL_DATA_SIMPLE = 1;

const int MODEL_GEOMETRY_PROCEDURAL = 2;
const int MODEL_DATA_PROCEDURAL = 2;

const int OCTREE_LOAD_RAW = 0;
const int OCTREE_LOAD_SPLIT = 1;
const int OCTREE_LOAD_PACKED = 2;

struct VoxelModel {
    DMirGeometry* geometry;
    DMirVoxelData* data;
    int geometry_type;
    int data_type;
    std::vector<DMirAddress> roots;
};

struct ProceduralParameters {
    double animation_time;
    float animation_speed;
    float terrain_detail;
    float terrain_height;
    float sphere_radius;
    struct vec3 sphere_center;
};

void delete_model(VoxelModel* model) {
    if (model->data_type == MODEL_DATA_SIMPLE) {
        delete (DMirSimpleData*)model->data;
    } else if (model->data_type == MODEL_DATA_PROCEDURAL) {
        delete (ProceduralVoxelData*)model->data;
    }
    
    if (model->geometry_type == MODEL_GEOMETRY_OCTREE) {
        auto octree = (DMirOctree*)model->geometry;
        delete octree->addr;
        delete octree;
    } else if (model->geometry_type == MODEL_GEOMETRY_PROCEDURAL) {
        auto proc_geo = (ProceduralGeometry*)model->geometry;
        delete (uint8_t*)proc_geo->parameters;
        delete proc_geo;
    }
    
    delete model;
}

float procedural_heightmap(float x, float y) {
    int32_t ix = (int32_t)x;
    int32_t iy = (int32_t)y;
    float tx = x - ix;
    float ty = y - iy;
    int x0 = (ix + 0);
    int x1 = (ix + 1);
    int y0 = (iy + 0);
    int y1 = (iy + 1);
    uint32_t k0 = (0x01000193 ^ x0) * 0x01000193;
    uint32_t k1 = (0x01000193 ^ x1) * 0x01000193;
    uint32_t k00 = (k0 ^ y0*2166136261) * 0x01000193;
    uint32_t k01 = (k1 ^ y0*2166136261) * 0x01000193;
    uint32_t k10 = (k0 ^ y1*2166136261) * 0x01000193;
    uint32_t k11 = (k1 ^ y1*2166136261) * 0x01000193;
    float h_scale = 1.0f / (float)0xFFFF;
    float h00 = (k00 & 0xFFFF) * h_scale;
    float h01 = (k01 & 0xFFFF) * h_scale;
    float h10 = (k10 & 0xFFFF) * h_scale;
    float h11 = (k11 & 0xFFFF) * h_scale;
    float txm = (3 - 2*tx)*tx*tx;
    float tym = (3 - 2*ty)*ty*ty;
    float h0 = h00 * (1 - txm) + h01 * txm;
    float h1 = h10 * (1 - txm) + h11 * txm;
    return h0 * (1 - tym) + h1 * tym;
}

int32_t procedural_sampler(void* parameters, float* position, float radius, uint8_t* color) {
    ProceduralParameters* params = (ProceduralParameters*)parameters;
    
    float terrain_x = (position[0] + params->animation_time) * params->terrain_detail;
    float terrain_y = (position[2] + params->animation_time) * params->terrain_detail;
    float terrain_height = params->terrain_height * procedural_heightmap(terrain_x, terrain_y);
    // Height difference is not a Euclidean distance, so use a larger margin
    float terrain_distance = (position[1] - terrain_height) - radius * 2;
    
    float distance_to_center = vec3_distance(position, (float*)(&params->sphere_center));
    float sphere_distance = (distance_to_center - params->sphere_radius) - radius;
    
    float v = position[1];
    color[0] = (uint8_t)(v*v * 255.0f);
    color[1] = (uint8_t)(v * 255.0f);
    color[2] = (uint8_t)(v * 0.0f);
    
    // // "Thin shell" (skip nodes which are fully inside the volume)
    // float diameter = 2*radius;
    // int terrain_side = ((terrain_distance <= 0) & (terrain_distance > (-2*diameter)) ? 1 : -1);
    // int sphere_side = ((sphere_distance <= 0) & (sphere_distance > (-diameter)) ? 1 : -1);
    // return std::max(terrain_side, sphere_side);
    
    // // "Heterogenous LOD" (splat as leaves the nodes which are fully inside the volume)
    // float diameter = 2*radius;
    // float dist_shell = std::min(terrain_distance, sphere_distance);
    // float dist_inside = std::min(terrain_distance + 2*diameter, sphere_distance + diameter);
    // if (dist_shell > 0) return -1;
    // return (dist_inside <= 0 ? 0 : 1);
    
    // "Volumetric" (traverse nodes which are fully inside the volume)
    float dist = std::min(terrain_distance, sphere_distance);
    return (dist <= 0 ? 1 : -1);
}

UPtr<VoxelModel> load_octree(DMirLookups* lookups, std::string path, int mode = OCTREE_LOAD_PACKED) {
    size_t file_size = 0;
    char* file_data = nullptr;
    read_file(path, file_size, &file_data);
    
    // This demo expects each node layout to be:
    // uint32 address + uint8 mask + uint8*3 rgb
    const int node_size = 8;
    
    if ((file_data == nullptr) || (file_size < 8*node_size)) {
        return UPtr<VoxelModel>(nullptr, nullptr);
    }
    
    auto voxel_count = file_size / node_size;
    
    if (voxel_count > UINT32_MAX) {
        // Too big; cannot be adequately represented in this implementation
        return UPtr<VoxelModel>(nullptr, nullptr);
    }
    
    auto voxel_data = new DMirSimpleData();
    voxel_data->api.get = simple_data_get_rgb;
    voxel_data->api.data_count = voxel_count;
    voxel_data->api.item_size = 3;
    voxel_data->items_stride = 8;
    voxel_data->items = ((uint8_t*)file_data) + 5;
    voxel_data->palette = nullptr;
    voxel_data->palette_count = 0;
    
    auto octree = new DMirOctree();
    octree->is_packed = false;
    octree->count = voxel_count;
    octree->addr = (uint32_t*)file_data;
    octree->mask = ((uint8_t*)file_data) + 4;
    octree->addr_stride = 8;
    octree->mask_stride = 8;
    octree->lookups = lookups;
    octree->geometry.traverse_start = octree_traverse_start;
    octree->geometry.traverse_next = octree_traverse_next;
    octree->geometry.evaluate = nullptr;
    
    auto model = UPtr<VoxelModel>(new VoxelModel(), delete_model);
    model->data = (DMirVoxelData*)voxel_data;
    model->data_type = MODEL_DATA_SIMPLE;
    model->geometry = (DMirGeometry*)octree;
    model->geometry_type = MODEL_GEOMETRY_OCTREE;
    model->roots.push_back(0);
    
    if (mode != OCTREE_LOAD_RAW) {
        char* new_data = new char[octree->count * (4 + 1 + 3)];
        uint32_t* addr = (uint32_t*)new_data;
        uint8_t* mask = (uint8_t*)(((char*)addr) + octree->count*4);
        RGB24* color = (RGB24*)(mask + octree->count*1);
        
        if (mode == OCTREE_LOAD_PACKED) {
            int count = 1;
            addr[0] = 0;
            
            for (int index = 0; index < count; index++) {
                auto node_address = *PTR_INDEX(octree->addr, addr[index]);
                auto node_mask = *PTR_INDEX(octree->mask, addr[index]);
                auto node_color = *((RGB24*)PTR_INDEX(voxel_data->items, addr[index]));
                addr[index] = count;
                mask[index] = node_mask;
                color[index] = node_color;
                
                for (int octant = 0; octant < 8; octant++) {
                    if ((node_mask & (1 << octant)) == 0) continue;
                    
                    addr[count] = node_address + octant;
                    count++;
                    
                    if (count > octree->count) {
                        // Recursion detected, can't proceed
                        delete new_data;
                        return model;
                    }
                }
            }
            
            octree->count = count;
            octree->is_packed = true;
        } else {
            for (int i = 0; i < octree->count; i++) {
                addr[i] = *PTR_INDEX(octree->addr, i);
                mask[i] = *PTR_INDEX(octree->mask, i);
                color[i] = *((RGB24*)PTR_INDEX(voxel_data->items, i));
            }
        }
        
        delete file_data;
        
        octree->addr = addr;
        octree->mask = mask;
        octree->addr_stride = 4;
        octree->mask_stride = 1;
        
        voxel_data->items = (uint8_t*)color;
        voxel_data->items_stride = 3;
    }
    
    return model;
}

UPtr<VoxelModel> make_procedural_geometry(ProceduralSampler sampler, uint32_t params_size) {
    auto voxel_data = new ProceduralVoxelData();
    voxel_data->api.get = procedural_data_get_rgb;
    voxel_data->api.data_count = 0;
    voxel_data->api.item_size = 3;
    
    auto proc_geo = new ProceduralGeometry();
    proc_geo->geometry.traverse_start = nullptr;
    proc_geo->geometry.traverse_next = nullptr;
    proc_geo->geometry.evaluate = procedural_evaluate;
    proc_geo->parameters = new uint8_t[params_size];
    proc_geo->sampler = sampler;
    
    auto model = UPtr<VoxelModel>(new VoxelModel(), delete_model);
    model->data = (DMirVoxelData*)voxel_data;
    model->data_type = MODEL_DATA_PROCEDURAL;
    model->geometry = (DMirGeometry*)proc_geo;
    model->geometry_type = MODEL_GEOMETRY_PROCEDURAL;
    model->roots.push_back(0);
    
    return model;
}
