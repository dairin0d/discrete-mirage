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
    void (*get)(void* data, DMirAddress data_ref, void* out);
    uint32_t data_count;
    uint32_t item_size; // in bytes
} DMirVoxelData;

// ===================================================== //

// VOXEL DATA EXAMPLE: PROCEDURAL DATA

typedef struct ProceduralVoxelData {
    DMirVoxelData api;
} ProceduralVoxelData;

void procedural_data_get_rgb(void* data_ptr, DMirAddress data_ref, void* out) {
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

void simple_data_get(void* data_ptr, DMirAddress data_ref, void* out) {
    DMirSimpleData* data = (DMirSimpleData*)data_ptr;
    uint8_t* item = PTR_INDEX(data->items, data_ref);
    memcpy(out, item, data->api.item_size);
}

// This is somewhat faster than the general memcpy
void simple_data_get_rgb(void* data_ptr, DMirAddress data_ref, void* out) {
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

int32_t procedural_evaluate(void* geometry, DMirNodeBox* node_box, DMirAddress* data_ref) {
    const float sqrt3 = 1.7320508075688772f;
    
    float scale = 1.0f / (1 << node_box->level);
    float position[3];
    position[0] = (node_box->x + 0.5f) * scale;
    position[1] = (node_box->y + 0.5f) * scale;
    position[2] = (node_box->z + 0.5f) * scale;
    float radius = scale * (0.5f * sqrt3);
    
    *data_ref = 0;
    
    ProceduralGeometry* proc_geo = (ProceduralGeometry*)geometry;
    return proc_geo->sampler(proc_geo->parameters, position, radius, (uint8_t*)data_ref);
}

// ===================================================== //

// GEOMETRY EXAMPLE: OCTREE

typedef struct DMirOctree {
    DMirGeometry geometry;
    DMirLookups* lookups;
    uint32_t* addr;
    uint8_t* mask;
    uint32_t addr_stride; // in bytes
    uint32_t mask_stride; // in bytes
    uint32_t count;
    DMirBool is_packed;
} DMirOctree;

DMirBool octree_traverse_start(void* geometry, uint64_t node_ref, uint64_t data_ref, DMirTraversal* dst) {
    DMirOctree* octree = (DMirOctree*)geometry;
    
    #ifdef DMIR_VALIDATE_ADDRESSES
    if (node_ref >= octree->count) return false;
    #endif
    
    dst->mask[0] = *PTR_INDEX(octree->mask, node_ref);
    dst->node[0] = node_ref;
    dst->data[0] = node_ref;
    
    return true;
}

DMirBool octree_traverse_next(void* geometry, DMirTraversal* src, int32_t index, DMirTraversal* dst) {
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

#ifdef __cplusplus
}
#endif

#endif
