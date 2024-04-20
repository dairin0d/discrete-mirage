// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

#ifndef DISCRETE_MIRAGE
#define DISCRETE_MIRAGE

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct DiscreteMirageBuffers {
    uint32_t* node; // node indices
    uint32_t* object; // object indices
    int32_t* depth; // linear depth
    uint64_t* stencil; // stencil masks
    float* grids; // cage subdivision stack
    uint32_t node_stride; // in bytes
    uint32_t object_stride; // in bytes
    uint32_t depth_stride; // in bytes
    uint32_t width;
    uint32_t height;
};

struct DiscreteMirageView {
    int32_t xmin;
    int32_t ymin;
    int32_t xmax;
    int32_t ymax;
    float depth_range; // far - near
    float perspective; // Z scaling factor
    float max_error; // othro threshold
};

struct DiscreteMirageOctree {
    uint32_t* nodes;
    uint8_t* masks;
    uint32_t nodes_stride; // in bytes
    uint32_t masks_stride; // in bytes
    uint32_t node_count;
};

struct DiscreteMirageScene {
    float* cages;
    uint32_t* roots;
    uint32_t cages_stride; // in bytes
    uint32_t roots_stride; // in bytes
    uint32_t object_count;
};

struct DiscreteMirageBuffers discrete_mirage_make_buffers(uint32_t width, uint32_t height);

void discrete_mirage_free_buffers(struct DiscreteMirageBuffers buffers);

void discrete_mirage_clear_buffers(struct DiscreteMirageBuffers buffers);

int32_t discrete_mirage_render(
    struct DiscreteMirageBuffers buffers,
    struct DiscreteMirageView view,
    struct DiscreteMirageOctree octree,
    struct DiscreteMirageScene scene);

#ifdef __cplusplus
}
#endif

#endif
