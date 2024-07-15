// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

#ifndef DISCRETE_MIRAGE
#define DISCRETE_MIRAGE

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DMIR_DEPTH_INT32
// #define DMIR_DEPTH_FLOAT

#define DMIR_COORD_FIXED 16
// #define DMIR_COORD_FLOAT

// #define DMIR_ROW_POW2

// #define DMIR_MINMAX_IF
#define DMIR_MINMAX_TERNARY

// #define DMIR_INT_SIZE 32
// #define DMIR_INT_SIZE 64

#define DMIR_USE_ORTHO
#define DMIR_ORTHO_TRAVERSE_ALT

#define DMIR_USE_OCCLUSION
#define DMIR_SKIP_OCCLUDED_ROWS

// #define DMIR_STENCIL_BITS 16
// #define DMIR_STENCIL_BITS 32
#define DMIR_STENCIL_BITS 64
// #define DMIR_STENCIL_1D
// #define DMIR_STENCIL_2D
#define DMIR_STENCIL_EXACT
// #define DMIR_STENCIL_COARSE

// #define DMIR_USE_SPLAT_COLOR
#define DMIR_USE_SPLAT_DEFERRED
#define DMIR_USE_SPLAT_PIXEL
#define DMIR_USE_BLIT_AT_2X2

typedef uint8_t DMirBool;

#ifdef DMIR_DEPTH_INT32
typedef int32_t DMirDepth;
const int32_t DMIR_MAX_DEPTH = INT32_MAX >> 1;
#else
typedef float DMirDepth;
const float DMIR_MAX_DEPTH = INFINITY;
#endif

typedef struct DMirColor {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} DMirColor;

typedef struct DMirRect {
    int32_t min_x;
    int32_t min_y;
    int32_t max_x;
    int32_t max_y;
} DMirRect;

typedef struct DMirFrustum {
    float min_depth;
    float max_depth;
    float focal_extent;
    float focal_depth;
    float perspective;
} DMirFrustum;

// Note: data is technically supposed to be an opaque pointer,
// but here we're using uint8_t for easier pointer arithmetic.
typedef struct DMirOctree {
    uint32_t* addr;
    uint8_t* mask;
    uint8_t* data;
    uint32_t addr_stride;
    uint32_t mask_stride;
    uint32_t data_stride;
    uint32_t count;
    uint32_t max_level;
    DMirBool is_packed;
} DMirOctree;

typedef struct DMirEffects {
    int32_t max_level;
    float dilation_abs;
    float dilation_rel;
} DMirEffects;

typedef struct DMirAffineInfo {
    uint32_t group;
    DMirOctree* octree;
    float matrix[3*3];
} DMirAffineInfo;

typedef struct DMirVoxelRef {
    int32_t affine_id;
    uint32_t address;
} DMirVoxelRef;

typedef struct DMirFramebuffer {
    DMirDepth* depth;
    DMirVoxelRef* voxel;
    DMirColor* color;
    uint32_t size_x;
    uint32_t size_y;
    uint32_t row_shift;
    uint32_t stencil_size_x;
    uint32_t stencil_size_y;
    uint32_t stencil_count_x;
    uint32_t stencil_count_y;
} DMirFramebuffer;

typedef struct DMirBatcher {
    DMirRect viewport;
    DMirFrustum frustum;
    float split_factor;
    float affine_distortion;
    float ortho_distortion;
    DMirRect rect;
} DMirBatcher;

typedef struct DMirRenderer {
    DMirFramebuffer* framebuffer;
    DMirBatcher* batcher;
    DMirRect rect;
} DMirRenderer;

const struct DMirRect DMIR_RECT_EMPTY = {
    .min_x = INT32_MAX,
    .min_y = INT32_MAX,
    .max_x = INT32_MIN,
    .max_y = INT32_MIN,
};

const struct DMirFrustum DMIR_FRUSTUM_DEFAULT = {
    .min_depth = 0.001f,
    .max_depth = 10000.0f,
    .focal_extent = 1,
    .focal_depth = 1,
    .perspective = 0,
};

#ifdef DMIR_ROW_POW2
int dmir_row_size(DMirFramebuffer* framebuffer) {
    return 1 << framebuffer->row_shift;
}
int dmir_pixel_index(DMirFramebuffer* framebuffer, int x, int y) {
    return x + (y << framebuffer->row_shift);
}
#else
int dmir_row_size(DMirFramebuffer* framebuffer) {
    return framebuffer->size_x;
}
int dmir_pixel_index(DMirFramebuffer* framebuffer, int x, int y) {
    return x + (y * framebuffer->size_x);
}
#endif

DMirFramebuffer* dmir_framebuffer_make(uint32_t size_x, uint32_t size_y);
void dmir_framebuffer_free(DMirFramebuffer* framebuffer);
void dmir_framebuffer_resize(DMirFramebuffer* framebuffer, uint32_t size_x, uint32_t size_y);
void dmir_framebuffer_clear(DMirFramebuffer* framebuffer);

DMirBool dmir_is_occluded_quad(DMirFramebuffer* framebuffer, DMirRect rect, DMirDepth depth);

DMirBatcher* dmir_batcher_make(void);
void dmir_batcher_free(DMirBatcher* batcher);
void dmir_batcher_reset(DMirBatcher* batcher, DMirRect viewport, DMirFrustum frustum);
void dmir_batcher_add(DMirBatcher* batcher, DMirFramebuffer* framebuffer,
    uint32_t group, float* cage, DMirOctree* octree, uint32_t root, DMirEffects effects);
void dmir_batcher_sort(DMirBatcher* batcher);
void dmir_batcher_affine_get(DMirBatcher* batcher, DMirAffineInfo** affine_infos, uint32_t* count);

DMirRenderer* dmir_renderer_make(void);
void dmir_renderer_free(DMirRenderer* renderer);
void dmir_renderer_draw(DMirRenderer* renderer);

#ifdef __cplusplus
}
#endif

#endif
