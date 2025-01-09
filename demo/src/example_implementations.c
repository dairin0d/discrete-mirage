// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

#ifndef DISCRETE_MIRAGE_EXAMPLE_IMPLEMENTATIONS
#define DISCRETE_MIRAGE_EXAMPLE_IMPLEMENTATIONS

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <discrete_mirage.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===================================================== //

// GENERAL / COMMON DEFINITIONS:

// Enable this if you aren't sure that your model(s)
// contain valid node addresses everywhere
#define DMIR_VALIDATE_ADDRESSES

#define PTR_OFFSET(array, offset) ((typeof(array))(((char*)(array)) + (offset)))
#define PTR_INDEX(array, index) PTR_OFFSET((array), ((index) * (array##_stride)))

typedef struct DMirVoxelData {
    void (*get)(const void* data, DMirAddress data_ref, void* out);
    uint32_t data_count;
    uint32_t item_size; // in bytes
} DMirVoxelData;

// ===================================================== //

// VOXEL DATA EXAMPLE: PROCEDURAL DATA

typedef struct ProceduralVoxelData {
    DMirVoxelData api;
} ProceduralVoxelData;

void procedural_data_get_rgb(const void* data_ptr, DMirAddress data_ref, void* out) {
    uint8_t* bytes = (uint8_t*)(&data_ref);
    ((uint8_t*)out)[0] = bytes[0] ^ bytes[3] ^ bytes[6];
    ((uint8_t*)out)[1] = bytes[1] ^ bytes[4] ^ bytes[7];
    ((uint8_t*)out)[2] = bytes[2] ^ bytes[5];
}

// ===================================================== //

// VOXEL DATA EXAMPLE: SIMPLE DATA

typedef struct DMirSimpleData {
    DMirVoxelData api;
    uint8_t* palette;
    uint8_t* items;
    uint32_t items_stride; // in bytes
    uint32_t palette_count;
} DMirSimpleData;

void simple_data_get(const void* data_ptr, DMirAddress data_ref, void* out) {
    DMirSimpleData* data = (DMirSimpleData*)data_ptr;
    uint8_t* item = PTR_INDEX(data->items, data_ref);
    memcpy(out, item, data->api.item_size);
}

// This is somewhat faster than the general memcpy
void simple_data_get_rgb(const void* data_ptr, DMirAddress data_ref, void* out) {
    DMirSimpleData* data = (DMirSimpleData*)data_ptr;
    uint8_t* item = PTR_INDEX(data->items, data_ref);
    ((uint8_t*)out)[0] = item[0];
    ((uint8_t*)out)[1] = item[1];
    ((uint8_t*)out)[2] = item[2];
}

// ===================================================== //

// GEOMETRY EXAMPLE: PROCEDURAL

typedef int32_t (*ProceduralSampler)(void* parameters, float* position, float radius, uint8_t* color);

typedef struct ProceduralGeometry {
    DMirGeometry geometry;
    void* parameters;
    ProceduralSampler sampler;
} ProceduralGeometry;

int32_t procedural_evaluate(const void* geometry, DMirNodeBox* node_box, DMirAddress* data_ref) {
    const float sqrt3 = 1.7320508075688772f;
    
    float scale = 1.0f / (1 << node_box->level);
    float position[3];
    position[0] = (node_box->x + 0.5f) * scale;
    position[1] = (node_box->y + 0.5f) * scale;
    position[2] = (node_box->z + 0.5f) * scale;
    float radius = scale * (0.5f * sqrt3);
    
    *data_ref = 0;
    
    ProceduralGeometry* proc_geo = (ProceduralGeometry*)geometry;
    int32_t result = proc_geo->sampler(proc_geo->parameters, position, radius, (uint8_t*)data_ref);
    if (result < 0) return -1;
    return result;
}

// ===================================================== //

// GEOMETRY EXAMPLE: OCTREE

typedef struct DMirOctree {
    DMirGeometry geometry;
    uint32_t* addr;
    uint8_t* mask;
    uint32_t addr_stride; // in bytes
    uint32_t mask_stride; // in bytes
    uint32_t count;
    DMirBool is_packed;
} DMirOctree;

DMirBool octree_traverse_start(const void* geometry, uint64_t node_ref, uint64_t data_ref, DMirTraversal* dst) {
    DMirOctree* octree = (DMirOctree*)geometry;
    
    #ifdef DMIR_VALIDATE_ADDRESSES
    if (node_ref >= octree->count) return false;
    #endif
    
    dst->mask[0] = *PTR_INDEX(octree->mask, node_ref);
    dst->node[0] = node_ref;
    dst->data[0] = node_ref;
    
    return true;
}

DMirBool octree_traverse_next(const void* geometry, DMirTraversal* src, int32_t index, DMirTraversal* dst) {
    DMirOctree* octree = (DMirOctree*)geometry;
    
    uint8_t mask = src->mask[index];
    uint64_t node = *PTR_INDEX(octree->addr, src->node[index]);
    
    if (octree->is_packed) {
        for (int octant = 0; octant < 8; octant++) {
            if (mask & (1 << octant)) {
                #ifdef DMIR_VALIDATE_ADDRESSES
                if (node >= octree->count) return false;
                #endif
                dst->mask[octant] = *PTR_INDEX(octree->mask, node);
                dst->node[octant] = node;
                dst->data[octant] = node;
                node++;
            }
        }
    } else {
        for (int octant = 0; octant < 8; octant++) {
            if (mask & (1 << octant)) {
                #ifdef DMIR_VALIDATE_ADDRESSES
                if (node >= octree->count) return false;
                #endif
                dst->mask[octant] = *PTR_INDEX(octree->mask, node);
                dst->node[octant] = node;
                dst->data[octant] = node;
            }
            node++;
        }
    }
    
    return true;
}

// ===================================================== //

// GEOMETRY EXAMPLE: DAG (DIRECTED ACYCLIC GRAPH)

typedef struct DMirVoxelDAG {
    DMirGeometry geometry;
    uint8_t* nodes;
    uint32_t level_starts[DMIR_LEVEL_MAX];
    uint32_t level_count;
    DMirBool is_compact;
} DMirVoxelDAG;

inline uint8_t flip_mask(uint8_t mask, uint8_t flip) {
    if (flip & 0b001) mask = ((mask & 0b10101010) >> 1) | ((mask & 0b01010101) << 1);
    if (flip & 0b010) mask = ((mask & 0b11001100) >> 2) | ((mask & 0b00110011) << 2);
    if (flip & 0b100) mask = ((mask & 0b11110000) >> 4) | ((mask & 0b00001111) << 4);
    return mask;
}

DMirBool dag_traverse_start(const void* geometry, uint64_t node_ref, uint64_t data_ref, DMirTraversal* dst) {
    DMirVoxelDAG* dag = (DMirVoxelDAG*)geometry;
    
    uint64_t level = node_ref >> (64-6);
    uint64_t flip = node_ref & 7;
    uint64_t node = (node_ref >> 3) & (UINT64_MAX >> (3+6));
    
    // In the traversal, we can actually start from below
    // level 2 due to perspective/deformation subdivision
    if (level == 0) {
        dst->mask[0] = 0;
        dst->node[0] = node_ref;
        dst->data[0] = data_ref;
    } else if (level <= 2) {
        uint8_t* node_buf = (uint8_t*)(dag->nodes + dag->level_starts[0]) + node;
        dst->mask[0] = flip_mask(node_buf[0], flip);
        dst->node[0] = node_ref;
        dst->data[0] = data_ref;
    } else if (!dag->is_compact) {
        uint32_t* node_buf = (uint32_t*)(dag->nodes + dag->level_starts[level-2]) + node;
        dst->mask[0] = flip_mask(node_buf[0], flip);
        dst->node[0] = node_ref;
        dst->data[0] = data_ref;
    } else {
        uint16_t* node_buf = (uint16_t*)(dag->nodes + dag->level_starts[level-2]) + node;
        dst->mask[0] = flip_mask(node_buf[0] & 255, flip);
        dst->node[0] = node_ref;
        dst->data[0] = data_ref;
    }
    
    return true;
}

DMirBool dag_traverse_next(const void* geometry, DMirTraversal* src, int32_t index, DMirTraversal* dst) {
    DMirVoxelDAG* dag = (DMirVoxelDAG*)geometry;
    
    uint64_t node_ref = src->node[index];
    uint64_t level = node_ref >> (64-6);
    uint64_t flip = node_ref & 7;
    uint64_t node = (node_ref >> 3) & (UINT64_MAX >> (3+6));
    
    uint8_t mask = src->mask[index];
    uint64_t data_ref = src->data[index];
    
    uint64_t sublevel = level - 1;
    
    if (level == 1) {
        uint64_t subnode_ref = (sublevel << (64-6)) | flip;
        
        for (uint64_t flip_octant = 0; flip_octant < 8; flip_octant++) {
            uint64_t octant = flip_octant ^ flip;
            
            if (mask & (1 << octant)) {
                dst->mask[octant] = 0;
                dst->node[octant] = subnode_ref;
                dst->data[octant] = 1;
            }
        }
    } else if (level == 2) {
        uint8_t* node_buf = (uint8_t*)(dag->nodes + dag->level_starts[0]) + node;
        
        node_buf++; // skip the mask
        node_buf++; // skip the subtree size info
        
        uint64_t subnode_pos = node + 2;
        
        for (uint64_t flip_octant = 0; flip_octant < 8; flip_octant++) {
            uint64_t octant = flip_octant ^ flip;
            
            if (mask & (1 << octant)) {
                uint64_t subnode_ref = (sublevel << (64-6)) | (subnode_pos << 3) | flip;
                
                dst->mask[octant] = flip_mask(node_buf[0], flip);
                dst->node[octant] = subnode_ref;
                
                dst->data[octant] = 1;
                for (uint8_t submask = node_buf[0]; submask; submask >>= 1) {
                    dst->data[octant] += (submask & 1);
                }
                
                node_buf++;
                subnode_pos++;
            }
        }
    } else if (!dag->is_compact) {
        uint32_t* node_buf = (uint32_t*)(dag->nodes + dag->level_starts[level-2]) + node;
        
        node_buf++; // skip the mask
        node_buf++; // skip the subtree size info
        
        for (uint64_t flip_octant = 0; flip_octant < 8; flip_octant++) {
            uint64_t octant = flip_octant ^ flip;
            
            if (mask & (1 << octant)) {
                uint64_t subnode_ref = (sublevel << (64-6)) | ((node_buf++)[0] ^ flip);
                
                dst->node[octant] = subnode_ref;
                
                uint64_t subnode = (subnode_ref >> 3) & (UINT64_MAX >> (3+6));
                uint64_t subflip = subnode_ref & 7;
                
                if (sublevel == 2) {
                    uint8_t* subnode_buf = (uint8_t*)(dag->nodes + dag->level_starts[0]) + subnode;
                    
                    dst->mask[octant] = flip_mask(subnode_buf[0], subflip);
                    
                    dst->data[octant] = subnode_buf[1];
                } else {
                    uint32_t* subnode_buf = (uint32_t*)(dag->nodes + dag->level_starts[sublevel-2]) + subnode;
                    
                    dst->mask[octant] = flip_mask((subnode_buf++)[0], subflip);
                    
                    dst->data[octant] = (subnode_buf++)[0];
                }
            }
        }
    } else {
        uint16_t* node_buf = (uint16_t*)(dag->nodes + dag->level_starts[level-2]) + node;
        
        uint8_t address_sizes = (node_buf++)[0] >> 8;
        
        uint64_t subtree_size = 0;
        uint64_t shift = 0;
        while (node_buf[0] & (1 << 15)) {
            subtree_size |= (node_buf++)[0] & (UINT16_MAX >> 1) << shift;
            shift += 15;
        }
        subtree_size |= (node_buf++)[0] << shift;
        
        uint32_t index = 0;
        for (uint64_t flip_octant = 0; flip_octant < 8; flip_octant++) {
            uint64_t octant = flip_octant ^ flip;
            
            if (mask & (1 << octant)) {
                uint64_t subnode_ref = (sublevel << (64-6)) | ((node_buf++)[0] ^ flip);
                if (address_sizes & (1 << index)) {
                    subnode_ref |= (node_buf++)[0] << 16;
                }
                
                dst->node[octant] = subnode_ref;
                
                uint64_t subnode = (subnode_ref >> 3) & (UINT64_MAX >> (3+6));
                uint64_t subflip = subnode_ref & 7;
                
                if (sublevel == 2) {
                    uint8_t* subnode_buf = (uint8_t*)(dag->nodes + dag->level_starts[0]) + subnode;
                    
                    dst->mask[octant] = flip_mask(subnode_buf[0], subflip);
                    
                    dst->data[octant] = subnode_buf[1];
                } else {
                    uint16_t* subnode_buf = (uint16_t*)(dag->nodes + dag->level_starts[sublevel-2]) + subnode;
                    
                    dst->mask[octant] = flip_mask((subnode_buf++)[0] & 255, subflip);
                    
                    subtree_size = 0;
                    shift = 0;
                    while (subnode_buf[0] & (1 << 15)) {
                        subtree_size |= (subnode_buf++)[0] & (UINT16_MAX >> 1) << shift;
                        shift += 15;
                    }
                    subtree_size |= (subnode_buf++)[0] << shift;
                    
                    dst->data[octant] = subtree_size;
                }
                
                index++;
            }
        }
    }
    
    // Now that we have all child data in voxel order,
    // we can calculate the actual voxel addresses
    for (uint64_t octant = 0; octant < 8; octant++) {
        if (mask & (1 << octant)) {
            uint64_t data_ref_next = data_ref + dst->data[octant];
            dst->data[octant] = data_ref + 1;
            data_ref = data_ref_next;
        }
    }
    
    return true;
}

// ===================================================== //

#ifdef __cplusplus
}
#endif

#endif
