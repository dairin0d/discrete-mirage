// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "discrete_mirage.h"

// NULL is defined in <stdlib.h>
#define FALSE 0
#define TRUE 1

// Compiler complains if array field's size is defined
// by a const variable, so we have to use a define
#define CAGE_SIZE 2*2*2
#define GRID_SIZE 3*3*3

#define GRID2(x, y, z) ((x) + 2*((y) + 2*(z)))
#define GRID3(x, y, z) ((x) + 3*((y) + 3*(z)))

#define ABS(value) ((value) < 0 ? -(value) : (value))

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) < (b) ? (b) : (a))
#define CLAMP(value, low, high) ((value) > (low) ? ((value) < (high) ? (value) : (high)) : (low))

#ifdef DMIR_MINMAX_IF
#define MIN_UPDATE(var, value) if ((value) < var) var = (value);
#define MAX_UPDATE(var, value) if ((value) > var) var = (value);
#define ABS_UPDATE(var) if (var < 0) var = -var;
#else
#define MIN_UPDATE(var, value) var = ((value) < var ? (value) : var);
#define MAX_UPDATE(var, value) var = ((value) > var ? (value) : var);
#define ABS_UPDATE(var) var = (var < 0 ? -var : var);
#endif

// 32-bit float screen-space coordinates can be halved
// at most 128 times before they become subpixel
#define MAX_STACK_DEPTH 128

// We have to limit the maximum screen-space size
// of relative dilation, because in perspective
// it can become too big near the eye plane
#define DILATION_REL_SIZE_MAX 256

#if DMIR_INT_SIZE == 32
typedef int32_t SInt;
typedef uint32_t UInt;
#elif DMIR_INT_SIZE == 64
typedef int64_t SInt;
typedef uint64_t UInt;
#else
typedef int_fast32_t SInt;
typedef uint_fast32_t UInt;
#endif

#ifdef DMIR_COORD_FIXED
typedef int32_t Coord;
#define SUBPIXEL_BITS DMIR_COORD_FIXED
#define SUBPIXEL_SIZE (1 << SUBPIXEL_BITS)
#define SUBPIXEL_HALF (SUBPIXEL_SIZE >> 1)
#define ORTHO_MAX_SUBDIVISIONS (31 - SUBPIXEL_BITS)
#define ORTHO_LEVEL_LIMIT (28 - SUBPIXEL_BITS)
#define ORTHO_MAX_SIZE (1 << (ORTHO_LEVEL_LIMIT - 1))
#define COORD_TO_PIXEL(coord) ((coord) >> SUBPIXEL_BITS)
#define PIXEL_TO_COORD(pixel) (((pixel) << SUBPIXEL_BITS) + SUBPIXEL_HALF)
#define COORD_HALVE(coord) ((coord) >> 1)
#else
typedef float Coord;
#define SUBPIXEL_SIZE 1.0f
#define SUBPIXEL_HALF 0.5f
#define ORTHO_MAX_SUBDIVISIONS 31
#define ORTHO_LEVEL_LIMIT 31
#define ORTHO_MAX_SIZE INT32_MAX
#define COORD_TO_PIXEL(coord) ((int32_t)(coord))
#define PIXEL_TO_COORD(pixel) ((pixel) + 0.5f)
#define COORD_HALVE(coord) ((coord) * 0.5f)
#endif

#ifdef DMIR_DEPTH_INT32
#define DEPTH_HALVE(depth) ((depth) >> 1)
#else
#define DEPTH_HALVE(depth) ((depth) * 0.5f)
#endif

#if DMIR_STENCIL_BITS == 16
typedef uint16_t Stencil;
#define STENCIL_CLEAR UINT16_MAX
#define STENCIL_BITS 16
#ifdef DMIR_STENCIL_1D
#define STENCIL_SHIFT_X 4
#define STENCIL_SHIFT_Y 0
#define STENCIL_LUT_STEP_X 1
#define STENCIL_LUT_STEP_Y 0
#define STENCIL_LUT_MASK_X 1
#define STENCIL_LUT_MASK_Y 0
#else
#define STENCIL_SHIFT_X 2
#define STENCIL_SHIFT_Y 2
#define STENCIL_LUT_STEP_X 1
#define STENCIL_LUT_STEP_Y 4
#define STENCIL_LUT_MASK_X 0x1111
#define STENCIL_LUT_MASK_Y 0xF
#endif
#elif DMIR_STENCIL_BITS == 32
typedef uint32_t Stencil;
#define STENCIL_CLEAR UINT32_MAX
#define STENCIL_BITS 32
#ifdef DMIR_STENCIL_1D
#define STENCIL_SHIFT_X 5
#define STENCIL_SHIFT_Y 0
#define STENCIL_LUT_STEP_X 1
#define STENCIL_LUT_STEP_Y 0
#define STENCIL_LUT_MASK_X 1
#define STENCIL_LUT_MASK_Y 0
#else
#define STENCIL_SHIFT_X 3
#define STENCIL_SHIFT_Y 2
#define STENCIL_LUT_STEP_X 1
#define STENCIL_LUT_STEP_Y 4
#define STENCIL_LUT_MASK_X 0x01010101
#define STENCIL_LUT_MASK_Y 0xFF
#endif
#elif DMIR_STENCIL_BITS == 64
typedef uint64_t Stencil;
#define STENCIL_CLEAR UINT64_MAX
#define STENCIL_BITS 64
#ifdef DMIR_STENCIL_1D
#define STENCIL_SHIFT_X 6
#define STENCIL_SHIFT_Y 0
#define STENCIL_LUT_STEP_X 1
#define STENCIL_LUT_STEP_Y 0
#define STENCIL_LUT_MASK_X 1
#define STENCIL_LUT_MASK_Y 0
#else
#define STENCIL_SHIFT_X 3
#define STENCIL_SHIFT_Y 3
#define STENCIL_LUT_STEP_X 1
#define STENCIL_LUT_STEP_Y 8
#define STENCIL_LUT_MASK_X UINT64_C(0x0101010101010101)
#define STENCIL_LUT_MASK_Y UINT64_C(0xFF)
#endif
#else
typedef uint32_t Stencil;
#define STENCIL_CLEAR 0
#define STENCIL_BITS 0
#define STENCIL_SHIFT_X 0
#define STENCIL_SHIFT_Y 0
#define STENCIL_LUT_STEP_X 0
#define STENCIL_LUT_STEP_Y 0
#define STENCIL_LUT_MASK_X 0
#define STENCIL_LUT_MASK_Y 0
#endif

#if defined(DMIR_STENCIL_LOCAL) && (STENCIL_BITS > 0)
#define STENCIL_LOCAL_SIZE STENCIL_BITS
#endif

#define STENCIL_SIZE_X (1 << STENCIL_SHIFT_X)
#define STENCIL_SIZE_Y (1 << STENCIL_SHIFT_Y)
#define STENCIL_MASK_X (STENCIL_SIZE_X - 1)
#define STENCIL_MASK_Y (STENCIL_SIZE_Y - 1)
#define STENCIL_AREA (STENCIL_SIZE_X * STENCIL_SIZE_Y)

#define STENCIL_UPDATE_STENCIL (1 << 31)
#define STENCIL_UPDATE_DEPTH (1 << 30)
#define STENCIL_UPDATE_NEXT ((1 << 30) - 1)

#define STENCIL_UPDATE_STENCIL_GET(tile) (\
    (tile)->update & STENCIL_UPDATE_STENCIL\
)
#define STENCIL_UPDATE_DEPTH_GET(tile) (\
    (tile)->update & STENCIL_UPDATE_DEPTH\
)
#define STENCIL_UPDATE_NEXT_GET(tile) (\
    (tile)->update & STENCIL_UPDATE_NEXT\
)

#define STENCIL_UPDATE_STENCIL_SET(tile) {\
    (tile)->update |= STENCIL_UPDATE_STENCIL;\
}
#define STENCIL_UPDATE_DEPTH_SET(tile) {\
    (tile)->update |= STENCIL_UPDATE_DEPTH;\
}
#define STENCIL_UPDATE_NEXT_SET(tile, value) {\
    (tile)->update = ((tile)->update & ~STENCIL_UPDATE_NEXT) | (value & STENCIL_UPDATE_NEXT);\
}

#define STENCIL_UPDATE_DEPTH_UNSET(tile) {\
    (tile)->update &= ~STENCIL_UPDATE_DEPTH;\
}

#if defined(DMIR_MAP_PRECISION) && defined(DMIR_COORD_FIXED)
#if defined(DMIR_USE_MAP_2) || defined(DMIR_USE_MAP_3) || defined(DMIR_USE_MAP_4)
#define USE_MAP
#endif
#endif

#if defined(USE_MAP) && defined(DMIR_USE_MAP_2)
#define MAP_2 2
#else
#define MAP_2 -1
#endif

#if defined(USE_MAP) && defined(DMIR_USE_MAP_3)
#define MAP_3 3
#else
#define MAP_3 -1
#endif

#if defined(USE_MAP) && defined(DMIR_USE_MAP_4)
#define MAP_4 4
#else
#define MAP_4 -1
#endif

#ifdef USE_MAP
#define MAP_BITS DMIR_MAP_PRECISION
#else
#define MAP_BITS 0
#endif
#define MAP_SIZE (1 << MAP_BITS)

#ifdef DMIR_CALCULATE_STATS
#define STAT_INCREMENT(framebuffer, stat) (framebuffer)->api.stats[(stat)]++;
#else
#define STAT_INCREMENT(framebuffer, stat)
#endif

typedef DMirBool Bool;
typedef DMirDepth Depth;
typedef DMirAddress Address;
typedef DMirAffineID AffineID;

typedef DMirQueue Queue;
typedef DMirLookups Lookups;
typedef DMirTraversal Traversal;
typedef DMirNodeBox NodeBox;
typedef DMirGeometry Geometry;

typedef DMirRect Rect;
typedef DMirFrustum Frustum;

typedef DMirGeometry Geometry;
typedef DMirEffects Effects;
typedef DMirAffineInfo AffineInfo;
typedef DMirVoxelRef VoxelRef;

typedef DMirFramebuffer Framebuffer;
typedef DMirBatcher Batcher;
typedef DMirRenderer Renderer;

typedef struct Vector2F {
    float x, y;
} Vector2F;

typedef struct Vector3F {
    float x, y, z;
} Vector3F;

typedef struct Vector3I {
    int32_t x, y, z;
} Vector3I;

// "S" stands for "screen-space"
typedef struct Vector3S {
    Coord x;
    Coord y;
    DMirDepth z;
} Vector3S;

typedef struct ProjectedVertex {
    Vector3F position;
    Vector2F projection;
    float scale;
} ProjectedVertex;

typedef struct Bounds {
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
} Bounds;

typedef struct StencilTile {
    Stencil self;
    Stencil scene;
    Depth depth;
    uint32_t update;
} StencilTile;

typedef struct GridStackItem {
    ProjectedVertex grid[GRID_SIZE];
    Rect rect;
    NodeBox node_box;
    Traversal traversal;
    uint32_t queue;
    SInt level;
    AffineID affine_id;
    SInt is_behind;
    SInt is_cube;
} GridStackItem;

typedef struct OrthoStackItem {
    Vector3S deltas[CAGE_SIZE];
    Vector3S center;
    Vector3S extent;
    Depth min_z;
    Depth max_z;
    Rect rect;
    NodeBox node_box;
    Traversal traversal;
    uint32_t queue;
    SInt level;
    SInt is_cube;
} OrthoStackItem;

typedef struct Fragment {
    int32_t x;
    int32_t y;
    Depth z;
    Address data_ref;
} Fragment;

typedef struct Subtree {
    AffineID affine_id;
    ProjectedVertex cage[CAGE_SIZE];
    NodeBox node_box;
    uint64_t node_ref;
    uint64_t data_ref;
    Effects effects;
    Bounds bounds;
    Rect rect;
    SInt next;
} Subtree;

#ifdef DMIR_ROW_POW2
#define PIXEL_INDEX(buf, x, y) ((x) + ((y) << (buf)->api.row_shift))
#else
#define PIXEL_INDEX(buf, x, y) ((x) + ((y) * (buf)->api.size_x))
#endif

UInt pow2_ceil(UInt value) {
    UInt shift = 0;
    while ((1 << shift) < value) shift++;
    return shift;
}

///////////////////
// Lookup tables //
///////////////////

#define XYZ DMIR_XYZ
#define XZY DMIR_XZY
#define YXZ DMIR_YXZ
#define YZX DMIR_YZX
#define ZXY DMIR_ZXY
#define ZYX DMIR_ZYX

// We could pre-compute the lookup tables and store
// them directly in the library, but then its size
// would significantly increase. By declaring the
// tables without initialization, we take advantage
// of BSS (Block Started by Symbol) mechanism, where
// the memory is allocated on library load. It still
// requires the user to initialize the tables as an
// explicit step, but we get the benefits of global
// access and don't need to free the memory manually.
Lookups dmir_lookups;

#define lookups dmir_lookups

void lookups_initialize(void) {
    // counts, octants, indices
    for (SInt mask = 0; mask < 256; mask++) {
        uint8_t count = 0;
        uint32_t o2i = 0, i2o = 0;
        for (uint32_t octant = 0; octant < 8; octant++) {
            if ((mask & (1 << octant)) == 0) continue;
            uint32_t index = count;
            o2i |= (index | 0b1000) << (octant*4);
            i2o |= (octant | 0b1000) << (index*4);
            count++;
        }
        lookups.counts[mask] = count;
        lookups.indices[mask] = o2i;
        lookups.octants[mask] = i2o;
    }
    
    // sparse and packed queues
    for (SInt order = 0; order < 6; order++) {
        SInt x_shift = 0, y_shift = 0, z_shift = 0;
        switch (order) {
        case XYZ: x_shift = 0; y_shift = 1; z_shift = 2; break;
        case XZY: x_shift = 0; y_shift = 2; z_shift = 1; break;
        case YXZ: x_shift = 1; y_shift = 0; z_shift = 2; break;
        case YZX: x_shift = 1; y_shift = 2; z_shift = 0; break;
        case ZXY: x_shift = 2; y_shift = 0; z_shift = 1; break;
        case ZYX: x_shift = 2; y_shift = 1; z_shift = 0; break;
        }
        
        for (SInt start_octant = 0; start_octant < 8; start_octant++) {
            for (SInt mask = 0; mask < 256; mask++) {
                uint32_t o2i = lookups.indices[mask];
                Queue queue = {.octants = 0, .indices = 0};
                SInt shift = 0;
                for (SInt z = 0; z <= 1; z++) {
                    for (SInt y = 0; y <= 1; y++) {
                        for (SInt x = 0; x <= 1; x++) {
                            SInt flip = (x << x_shift) | (y << y_shift) | (z << z_shift);
                            SInt octant = (start_octant ^ flip);
                            if ((mask & (1 << octant)) == 0) continue;
                            SInt index = (o2i >> (octant*4)) & 0b111;
                            queue.octants |= ((octant | 0b1000) << shift);
                            queue.indices |= ((index | 0b1000) << shift);
                            shift += 4;
                        }
                    }
                }
                lookups.packed[(((order << 3) | start_octant) << 8) | mask] = queue;
            }
        }
    }
    
    const int queues_count = 6*8*256;
    for (SInt i = 0; i < queues_count; i++) {
        lookups.sparse[i].octants = lookups.sparse[i].indices = lookups.packed[i].octants;
    }
    
    // flips
    for (SInt flip = 0; flip < 8; flip++) {
        for (SInt maskBase = 0; maskBase < 256; maskBase++) {
            uint8_t mask = maskBase;
            if (flip & 0b001) mask = ((mask & 0b10101010) >> 1) | ((mask & 0b01010101) << 1);
            if (flip & 0b010) mask = ((mask & 0b11001100) >> 2) | ((mask & 0b00110011) << 2);
            if (flip & 0b100) mask = ((mask & 0b11110000) >> 4) | ((mask & 0b00001111) << 4);
            lookups.flips[(flip << 8) | maskBase] = mask;
        }
    }
}

/////////////////////////////////////
// Rendering-related functionality //
/////////////////////////////////////

typedef struct FramebufferInternal {
    Framebuffer api;
    StencilTile* stencil_tiles;
    Stencil* stencil_x_base;
    Stencil* stencil_y_base;
    Stencil* stencil_x;
    Stencil* stencil_y;
    SInt stencil_row_shift;
} FramebufferInternal;

typedef struct BatcherInternal {
    Batcher api;
    AffineInfo* affine;
    Subtree* subtrees;
    GridStackItem* stack;
    SInt affine_size;
    SInt subtrees_size;
    SInt affine_count;
    SInt subtrees_count;
    SInt sorted_head;
    float scale_xy;
    float scale_c;
    float scale_z;
    float offset_x;
    float offset_y;
    float depth_factor;
    float eye_z;
    float clamp_z;
    SInt is_perspective;
    Bounds frustum_bounds;
} BatcherInternal;

typedef struct RendererInternal {
    Renderer api;
    GridStackItem* stack;
    Fragment* fragments;
    SInt fragments_size;
} RendererInternal;

#ifdef DMIR_DEPTH_INT32
static inline Depth z_to_depth(BatcherInternal* batcher, float z) {
    return (Depth)((z - batcher->api.frustum.min_depth) * batcher->depth_factor);
}
static inline Depth dz_to_depth(BatcherInternal* batcher, float dz) {
    return (Depth)(dz * batcher->depth_factor);
}
#else
static inline Depth z_to_depth(BatcherInternal* batcher, float z) {
    return (Depth)z;
}
static inline Depth dz_to_depth(BatcherInternal* batcher, float dz) {
    return (Depth)dz;
}
#endif

static inline void node_box_child(const NodeBox* src, SInt octant, NodeBox* dst) {
    dst->level = src->level + 1;
    dst->x = (src->x << 1) + ((octant & 0b001) != 0);
    dst->y = (src->y << 1) + ((octant & 0b010) != 0);
    dst->z = (src->z << 1) + ((octant & 0b100) != 0);
}

static SInt is_occluded_quad(FramebufferInternal* framebuffer, Rect* rect, Depth depth) {
    #ifndef DMIR_USE_OCCLUSION
    return FALSE;
    #endif
    
    #if STENCIL_BITS > 0
    SInt min_tx = rect->min_x & ~STENCIL_MASK_X;
    SInt max_tx = rect->max_x & ~STENCIL_MASK_X;
    SInt min_ty = rect->min_y & ~STENCIL_MASK_Y;
    SInt max_ty = rect->max_y & ~STENCIL_MASK_Y;
    SInt tile_col = min_tx >> STENCIL_SHIFT_X;
    SInt tile_row = min_ty >> STENCIL_SHIFT_Y;
    Stencil* stencil_x = framebuffer->stencil_x;
    Stencil* stencil_y = framebuffer->stencil_y;
    for (SInt ty = min_ty; ty <= max_ty; ty += STENCIL_SIZE_Y, tile_row++) {
        StencilTile* tile = framebuffer->stencil_tiles +
            (tile_col + (tile_row << framebuffer->stencil_row_shift));
        for (SInt tx = min_tx; tx <= max_tx; tx += STENCIL_SIZE_X, tile++) {
            Stencil pixel_mask = tile->self;
            pixel_mask &=
                (stencil_x[rect->min_x - tx] ^ stencil_x[rect->max_x - tx + 1]) &
                (stencil_y[rect->min_y - ty] ^ stencil_y[rect->max_y - ty + 1]);
            if (depth >= tile->depth) pixel_mask &= tile->scene;
            if (pixel_mask != 0) {
                STAT_INCREMENT(framebuffer, DMIR_OCCLUSIONS_PASSED);
                return FALSE;
            }
        }
        #ifdef DMIR_SKIP_OCCLUDED_ROWS
        rect->min_y = ty + STENCIL_SIZE_Y;
        #endif
    }
    #else
    for (SInt y = rect->min_y; y <= rect->max_y; y++) {
        SInt row = PIXEL_INDEX(framebuffer, 0, y);
        for (SInt x = rect->min_x; x <= rect->max_x; x++) {
            if (framebuffer->api.depth[row+x] > depth) {
                STAT_INCREMENT(framebuffer, DMIR_OCCLUSIONS_PASSED);
                return FALSE;
            }
        }
        #ifdef DMIR_SKIP_OCCLUDED_ROWS
        rect->min_y = y + 1;
        #endif
    }
    #endif
    
    STAT_INCREMENT(framebuffer, DMIR_OCCLUSIONS_FAILED);
    return TRUE;
}

// Compensate for projected bounds' inaccuracy
// Factor should be large enough to conservatively cover
// the error, but small enough to not impact performance
const float BOUNDS_EXPANSION_FACTOR = 1.0f / (1 << 24);
static inline void enlarge_bounds(Bounds* bounds) {
    float center_x = (bounds->min_x + bounds->max_x) * 0.5f;
    float center_y = (bounds->min_y + bounds->max_y) * 0.5f;
    bounds->min_x += (bounds->min_x - center_x) * BOUNDS_EXPANSION_FACTOR;
    bounds->min_y += (bounds->min_y - center_y) * BOUNDS_EXPANSION_FACTOR;
    bounds->max_x += (bounds->max_x - center_x) * BOUNDS_EXPANSION_FACTOR;
    bounds->max_y += (bounds->max_y - center_y) * BOUNDS_EXPANSION_FACTOR;
}

#define RECT_CLIP(rect, other) {\
    MAX_UPDATE((rect).min_x, (other).min_x);\
    MAX_UPDATE((rect).min_y, (other).min_y);\
    MIN_UPDATE((rect).max_x, (other).max_x);\
    MIN_UPDATE((rect).max_y, (other).max_y);\
}
#define RECT_EXPAND(rect, other) {\
    MIN_UPDATE((rect).min_x, (other).min_x);\
    MIN_UPDATE((rect).min_y, (other).min_y);\
    MAX_UPDATE((rect).max_x, (other).max_x);\
    MAX_UPDATE((rect).max_y, (other).max_y);\
}
#define RECT_FROM_BOUNDS(rect, bounds, dilation) {\
    (rect).min_x = (int32_t)CLAMP((bounds).min_x - (dilation), (INT32_MIN >> 2), (INT32_MAX >> 2));\
    (rect).max_x = (int32_t)CLAMP((bounds).max_x + (dilation), (INT32_MIN >> 2), (INT32_MAX >> 2));\
    (rect).min_y = (int32_t)CLAMP((bounds).min_y - (dilation), (INT32_MIN >> 2), (INT32_MAX >> 2));\
    (rect).max_y = (int32_t)CLAMP((bounds).max_y + (dilation), (INT32_MIN >> 2), (INT32_MAX >> 2));\
}
#define RECT_FROM_POINT(rect, position, extent_x, extent_y) {\
    (rect).min_x = COORD_TO_PIXEL((position).x - (extent_x));\
    (rect).max_x = COORD_TO_PIXEL((position).x + (extent_x));\
    (rect).min_y = COORD_TO_PIXEL((position).y - (extent_y));\
    (rect).max_y = COORD_TO_PIXEL((position).y + (extent_y));\
}

static inline void project(ProjectedVertex* vertex, BatcherInternal* batcher) {
    float clamped_z = MAX(vertex->position.z, batcher->clamp_z);
    vertex->scale = batcher->scale_xy / (batcher->scale_c + batcher->scale_z * clamped_z);
    vertex->projection.x = batcher->offset_x + vertex->position.x * vertex->scale;
    vertex->projection.y = batcher->offset_y + vertex->position.y * vertex->scale;
    // If not valid, assign NaN afterwards so that it's ignored in max scale calculations
    if (vertex->position.z <= batcher->clamp_z) vertex->scale = NAN;
}

#define GRID_CENTER(grid, component) (grid[GRID3(1,1,1)].component)
#define GRID_AXIS(grid, x, y, z, component) (\
    grid[GRID3(1+x,1+y,1+z)].component - grid[GRID3(1,1,1)].component\
)

#define GRID_CENTER_CAGE(grid, component) (\
    0.5f * (grid[GRID3(0,0,0)].component + grid[GRID3(2,2,2)].component)\
)
#define GRID_AXIS_CAGE(grid, x, y, z, component) 0.25f * (\
    (grid[GRID3(0+(x)*2,0+(y)*2,0+(z)*2)].component - grid[GRID3(0,0,0)].component) -\
    (grid[GRID3(2-(x)*2,2-(y)*2,2-(z)*2)].component - grid[GRID3(2,2,2)].component)\
)

#define CALC_DISTORTION(grid, x, y, z, component) fabsf(\
    (grid[GRID3(0+(x)*2,0+(y)*2,0+(z)*2)].component - grid[GRID3(0,0,0)].component) +\
    (grid[GRID3(2-(x)*2,2-(y)*2,2-(z)*2)].component - grid[GRID3(2,2,2)].component)\
)

float calculate_affine_distortion(ProjectedVertex* grid) {
    float distortion, max_distortion = 0.0f;
    distortion = CALC_DISTORTION(grid, 1, 0, 0, position.x);
    MAX_UPDATE(max_distortion, distortion);
    distortion = CALC_DISTORTION(grid, 1, 0, 0, position.y);
    MAX_UPDATE(max_distortion, distortion);
    distortion = CALC_DISTORTION(grid, 1, 0, 0, position.z);
    MAX_UPDATE(max_distortion, distortion);
    distortion = CALC_DISTORTION(grid, 0, 1, 0, position.x);
    MAX_UPDATE(max_distortion, distortion);
    distortion = CALC_DISTORTION(grid, 0, 1, 0, position.y);
    MAX_UPDATE(max_distortion, distortion);
    distortion = CALC_DISTORTION(grid, 0, 1, 0, position.z);
    MAX_UPDATE(max_distortion, distortion);
    distortion = CALC_DISTORTION(grid, 0, 0, 1, position.x);
    MAX_UPDATE(max_distortion, distortion);
    distortion = CALC_DISTORTION(grid, 0, 0, 1, position.y);
    MAX_UPDATE(max_distortion, distortion);
    distortion = CALC_DISTORTION(grid, 0, 0, 1, position.z);
    MAX_UPDATE(max_distortion, distortion);
    return max_distortion;
}

float calculate_projection_distortion(ProjectedVertex* grid) {
    float distortion, max_distortion = 0.0f;
    distortion = CALC_DISTORTION(grid, 1, 0, 0, projection.x);
    MAX_UPDATE(max_distortion, distortion);
    distortion = CALC_DISTORTION(grid, 1, 0, 0, projection.y);
    MAX_UPDATE(max_distortion, distortion);
    distortion = CALC_DISTORTION(grid, 0, 1, 0, projection.x);
    MAX_UPDATE(max_distortion, distortion);
    distortion = CALC_DISTORTION(grid, 0, 1, 0, projection.y);
    MAX_UPDATE(max_distortion, distortion);
    distortion = CALC_DISTORTION(grid, 0, 0, 1, projection.x);
    MAX_UPDATE(max_distortion, distortion);
    distortion = CALC_DISTORTION(grid, 0, 0, 1, projection.y);
    MAX_UPDATE(max_distortion, distortion);
    return max_distortion;
}

Vector3F normalized_matrix_axis(ProjectedVertex* grid, SInt x, SInt y, SInt z) {
    Vector3F axis = {
        .x = GRID_AXIS(grid, x, y, z, position.x),
        .y = GRID_AXIS(grid, x, y, z, position.y),
        .z = GRID_AXIS(grid, x, y, z, position.z),
    };
    float magnitude = axis.x*axis.x + axis.y*axis.y + axis.z*axis.z;
    if (magnitude > 0.0f) {
        magnitude = sqrtf(magnitude);
        axis.x /= magnitude;
        axis.y /= magnitude;
        axis.z /= magnitude;
    }
    return axis;
}

SInt calculate_starting_octant(ProjectedVertex* grid) {
    float xx = GRID_AXIS(grid, 1, 0, 0, projection.x);
    float xy = GRID_AXIS(grid, 1, 0, 0, projection.y);
    float yx = GRID_AXIS(grid, 0, 1, 0, projection.x);
    float yy = GRID_AXIS(grid, 0, 1, 0, projection.y);
    float zx = GRID_AXIS(grid, 0, 0, 1, projection.x);
    float zy = GRID_AXIS(grid, 0, 0, 1, projection.y);
    SInt bit_x = (yy * zx <= yx * zy ? 0 : 1);
    SInt bit_y = (zy * xx <= zx * xy ? 0 : 2);
    SInt bit_z = (xy * yx <= xx * yy ? 0 : 4);
    return bit_x | bit_y | bit_z;
}

SInt calculate_starting_octant_perspective(ProjectedVertex* grid, float eye_z) {
    float px = -GRID_CENTER(grid, position.x);
    float py = -GRID_CENTER(grid, position.y);
    float pz = -GRID_CENTER(grid, position.z) + eye_z;
    float xx = GRID_AXIS(grid, 1, 0, 0, position.x);
    float xy = GRID_AXIS(grid, 1, 0, 0, position.y);
    float xz = GRID_AXIS(grid, 1, 0, 0, position.z);
    float yx = GRID_AXIS(grid, 0, 1, 0, position.x);
    float yy = GRID_AXIS(grid, 0, 1, 0, position.y);
    float yz = GRID_AXIS(grid, 0, 1, 0, position.z);
    float zx = GRID_AXIS(grid, 0, 0, 1, position.x);
    float zy = GRID_AXIS(grid, 0, 0, 1, position.y);
    float zz = GRID_AXIS(grid, 0, 0, 1, position.z);
    float dot_x = (yy*zz - yz*zy)*px + (yz*zx - yx*zz)*py + (yx*zy - yy*zx)*pz;
    float dot_y = (zy*xz - zz*xy)*px + (zz*xx - zx*xz)*py + (zx*xy - zy*xx)*pz;
    float dot_z = (xy*yz - xz*yy)*px + (xz*yx - xx*yz)*py + (xx*yy - xy*yx)*pz;
    SInt bit_x = (dot_x <= 0 ? 0 : 1);
    SInt bit_y = (dot_y <= 0 ? 0 : 2);
    SInt bit_z = (dot_z <= 0 ? 0 : 4);
    return bit_x | bit_y | bit_z;
}

SInt calculate_starting_octant_ortho(Vector3F* matrix) {
    SInt bit_x = (matrix[1].y * matrix[2].x <= matrix[1].x * matrix[2].y ? 0 : 1);
    SInt bit_y = (matrix[2].y * matrix[0].x <= matrix[2].x * matrix[0].y ? 0 : 2);
    SInt bit_z = (matrix[0].y * matrix[1].x <= matrix[0].x * matrix[1].y ? 0 : 4);
    return bit_x | bit_y | bit_z;
}

SInt calculate_octant_order(Vector3F* matrix) {
    Depth xz = ABS(matrix[0].z);
    Depth yz = ABS(matrix[1].z);
    Depth zz = ABS(matrix[2].z);
    return (xz <= yz
        ? (xz <= zz ? (yz <= zz ? XYZ : XZY) : ZXY)
        : (yz <= zz ? (xz <= zz ? YXZ : YZX) : ZYX));
}

SInt calculate_octant_order_grid(ProjectedVertex* grid) {
    float xz = GRID_AXIS(grid, 1, 0, 0, position.z);
    ABS_UPDATE(xz);
    float yz = GRID_AXIS(grid, 0, 1, 0, position.z);
    ABS_UPDATE(yz);
    float zz = GRID_AXIS(grid, 0, 0, 1, position.z);
    ABS_UPDATE(zz);
    return (xz <= yz
        ? (xz <= zz ? (yz <= zz ? XYZ : XZY) : ZXY)
        : (yz <= zz ? (xz <= zz ? YXZ : YZX) : ZYX));
}

#define GRID_INIT(grid, cage, x, y, z, batcher) {\
    grid[GRID3(x,y,z)].position = cage[GRID2(x/2,y/2,z/2)];\
    project(&grid[GRID3(x,y,z)], batcher);\
}

#define GRID_FROM_CAGE(grid, cage, x, y, z) grid[GRID3(x,y,z)] = cage[GRID2(x/2,y/2,z/2)];
#define CAGE_FROM_GRID(grid, cage, x, y, z) cage[GRID2(x/2,y/2,z/2)] = grid[GRID3(x,y,z)];

#define VERT_MIDPOINT(vmin, vmax, vmid) {\
    vmid.position.x = (vmin.position.x + vmax.position.x) * 0.5f;\
    vmid.position.y = (vmin.position.y + vmax.position.y) * 0.5f;\
    vmid.position.z = (vmin.position.z + vmax.position.z) * 0.5f;\
}
#define GRID_MIDPOINT(grid, x0, y0, z0, x1, y1, z1, batcher) {\
    VERT_MIDPOINT(\
        grid[GRID3(x0,y0,z0)],\
        grid[GRID3(x1,y1,z1)],\
        grid[GRID3((x0+x1)/2,(y0+y1)/2,(z0+z1)/2)]);\
    project(&grid[GRID3((x0+x1)/2,(y0+y1)/2,(z0+z1)/2)], batcher);\
}

void initialize_grid(ProjectedVertex* grid, Vector3F* cage, BatcherInternal* batcher) {
    GRID_INIT(grid, cage, 0, 0, 0, batcher);
    GRID_INIT(grid, cage, 2, 0, 0, batcher);
    GRID_INIT(grid, cage, 0, 2, 0, batcher);
    GRID_INIT(grid, cage, 2, 2, 0, batcher);
    GRID_INIT(grid, cage, 0, 0, 2, batcher);
    GRID_INIT(grid, cage, 2, 0, 2, batcher);
    GRID_INIT(grid, cage, 0, 2, 2, batcher);
    GRID_INIT(grid, cage, 2, 2, 2, batcher);
}

void grid_from_cage(ProjectedVertex* grid, ProjectedVertex* cage) {
    GRID_FROM_CAGE(grid, cage, 0, 0, 0);
    GRID_FROM_CAGE(grid, cage, 2, 0, 0);
    GRID_FROM_CAGE(grid, cage, 0, 2, 0);
    GRID_FROM_CAGE(grid, cage, 2, 2, 0);
    GRID_FROM_CAGE(grid, cage, 0, 0, 2);
    GRID_FROM_CAGE(grid, cage, 2, 0, 2);
    GRID_FROM_CAGE(grid, cage, 0, 2, 2);
    GRID_FROM_CAGE(grid, cage, 2, 2, 2);
}

void cage_from_grid(ProjectedVertex* grid, ProjectedVertex* cage) {
    CAGE_FROM_GRID(grid, cage, 0, 0, 0);
    CAGE_FROM_GRID(grid, cage, 2, 0, 0);
    CAGE_FROM_GRID(grid, cage, 0, 2, 0);
    CAGE_FROM_GRID(grid, cage, 2, 2, 0);
    CAGE_FROM_GRID(grid, cage, 0, 0, 2);
    CAGE_FROM_GRID(grid, cage, 2, 0, 2);
    CAGE_FROM_GRID(grid, cage, 0, 2, 2);
    CAGE_FROM_GRID(grid, cage, 2, 2, 2);
}

void calculate_midpoints(ProjectedVertex* grid, BatcherInternal* batcher) {
    // X edges
    GRID_MIDPOINT(grid, 0,0,0, 2,0,0, batcher);
    GRID_MIDPOINT(grid, 0,2,0, 2,2,0, batcher);
    GRID_MIDPOINT(grid, 0,0,2, 2,0,2, batcher);
    GRID_MIDPOINT(grid, 0,2,2, 2,2,2, batcher);
    // Y edges
    GRID_MIDPOINT(grid, 0,0,0, 0,2,0, batcher);
    GRID_MIDPOINT(grid, 2,0,0, 2,2,0, batcher);
    GRID_MIDPOINT(grid, 0,0,2, 0,2,2, batcher);
    GRID_MIDPOINT(grid, 2,0,2, 2,2,2, batcher);
    // Z edges
    GRID_MIDPOINT(grid, 0,0,0, 0,0,2, batcher);
    GRID_MIDPOINT(grid, 2,0,0, 2,0,2, batcher);
    GRID_MIDPOINT(grid, 0,2,0, 0,2,2, batcher);
    GRID_MIDPOINT(grid, 2,2,0, 2,2,2, batcher);
    // Faces
    GRID_MIDPOINT(grid, 1,0,0, 1,2,0, batcher);
    GRID_MIDPOINT(grid, 0,1,0, 0,1,2, batcher);
    GRID_MIDPOINT(grid, 0,0,1, 2,0,1, batcher);
    GRID_MIDPOINT(grid, 1,0,2, 1,2,2, batcher);
    GRID_MIDPOINT(grid, 2,1,0, 2,1,2, batcher);
    GRID_MIDPOINT(grid, 0,2,1, 2,2,1, batcher);
    // Center
    GRID_MIDPOINT(grid, 1,1,0, 1,1,2, batcher);
}

void initialize_subgrid(ProjectedVertex* subgrid, ProjectedVertex* grid, SInt octant) {
    SInt ox = (octant >> 0) & 1;
    SInt oy = (octant >> 1) & 1;
    SInt oz = (octant >> 2) & 1;
    subgrid[GRID3(0,0,0)] = grid[GRID3(0+ox,0+oy,0+oz)];
    subgrid[GRID3(2,0,0)] = grid[GRID3(1+ox,0+oy,0+oz)];
    subgrid[GRID3(0,2,0)] = grid[GRID3(0+ox,1+oy,0+oz)];
    subgrid[GRID3(2,2,0)] = grid[GRID3(1+ox,1+oy,0+oz)];
    subgrid[GRID3(0,0,2)] = grid[GRID3(0+ox,0+oy,1+oz)];
    subgrid[GRID3(2,0,2)] = grid[GRID3(1+ox,0+oy,1+oz)];
    subgrid[GRID3(0,2,2)] = grid[GRID3(0+ox,1+oy,1+oz)];
    subgrid[GRID3(2,2,2)] = grid[GRID3(1+ox,1+oy,1+oz)];
}

void calculate_screen_bounds(ProjectedVertex* grid, Bounds* bounds, float* max_scale) {
    *max_scale = 0;
    bounds->min_x = INFINITY;
    bounds->max_x = -INFINITY;
    bounds->min_y = INFINITY;
    bounds->max_y = -INFINITY;
    bounds->min_z = INFINITY;
    bounds->max_z = -INFINITY;
    for (SInt iz = 0; iz < 3; iz += 2) {
        for (SInt iy = 0; iy < 3; iy += 2) {
            for (SInt ix = 0; ix < 3; ix += 2) {
                MAX_UPDATE(*max_scale, grid[GRID3(ix, iy, iz)].scale);
                MIN_UPDATE(bounds->min_x, grid[GRID3(ix, iy, iz)].projection.x);
                MAX_UPDATE(bounds->max_x, grid[GRID3(ix, iy, iz)].projection.x);
                MIN_UPDATE(bounds->min_y, grid[GRID3(ix, iy, iz)].projection.y);
                MAX_UPDATE(bounds->max_y, grid[GRID3(ix, iy, iz)].projection.y);
                MIN_UPDATE(bounds->min_z, grid[GRID3(ix, iy, iz)].position.z);
                MAX_UPDATE(bounds->max_z, grid[GRID3(ix, iy, iz)].position.z);
            }
        }
    }
}

#define STENCIL_INDEX(buf, x, y) (\
    ((x) >> STENCIL_SHIFT_X) + (((y) >> STENCIL_SHIFT_Y) << (buf)->stencil_row_shift)\
)
#define STENCIL_BIT(x, y) (\
    ((Stencil)(1)) << (((x) & STENCIL_MASK_X) + (((y) & STENCIL_MASK_Y) << STENCIL_SHIFT_X))\
)

static inline void write_pixel(FramebufferInternal* framebuffer, SInt i,
    AffineID affine_id, Address data_ref)
{
    STAT_INCREMENT(framebuffer, DMIR_FRAGMENTS_WRITTEN);
    framebuffer->api.voxel[i].affine_id = affine_id;
    framebuffer->api.voxel[i].voxel_id = data_ref;
}

static inline void add_fragment(Fragment** fragments, SInt x, SInt y, Depth depth, Address data_ref) {
    fragments[0]->x = x;
    fragments[0]->y = y;
    fragments[0]->z = depth;
    fragments[0]->data_ref = data_ref;
    fragments[0]++;
}

#if STENCIL_BITS > 0
#define CHECK_AND_WRITE_STENCIL(framebuffer, x, y, stop) {\
    SInt stencil_index = STENCIL_INDEX(framebuffer, x, y);\
    Stencil stencil_mask = STENCIL_BIT(x, y);\
    if (!(framebuffer->stencil_tiles[stencil_index].self & stencil_mask)) stop;\
    framebuffer->stencil_tiles[stencil_index].self &= ~stencil_mask;\
}
#else
#define CHECK_AND_WRITE_STENCIL(framebuffer, x, y, stop)
#endif

#define CHECK_AND_WRITE_DEPTH(framebuffer, x, y, z, stop) {\
    SInt i = PIXEL_INDEX(framebuffer, x, y);\
    if (!(z < framebuffer->api.depth[i])) stop;\
    framebuffer->api.depth[i] = z;\
}

#ifdef DMIR_USE_SPLAT_DEFERRED
#if STENCIL_BITS > 0
#define SPLAT(framebuffer, fragments, x, y, depth, affine_id, data_ref, stop) {\
    STAT_INCREMENT(framebuffer, DMIR_FRAGMENTS_ADDED);\
    add_fragment(&(fragments), x, y, depth, data_ref);\
}
#else
#define SPLAT(framebuffer, fragments, x, y, depth, affine_id, data_ref, stop) {\
    STAT_INCREMENT(framebuffer, DMIR_FRAGMENTS_ADDED);\
    CHECK_AND_WRITE_DEPTH(framebuffer, x, y, depth, stop);\
    add_fragment(&(fragments), x, y, depth, data_ref);\
}
#endif
#else
#define SPLAT(framebuffer, fragments, x, y, depth, affine_id, data_ref, stop) {\
    STAT_INCREMENT(framebuffer, DMIR_FRAGMENTS_ADDED);\
    CHECK_AND_WRITE_DEPTH(framebuffer, x, y, depth, stop);\
    {\
        SInt i = PIXEL_INDEX(framebuffer, x, y);\
        write_pixel(framebuffer, i, affine_id, data_ref);\
    }\
}
#endif

SInt calculate_max_level(Vector3F* fmatrix) {
    float gap_x = fabsf(fmatrix[0].x) + fabsf(fmatrix[1].x) + fabsf(fmatrix[2].x);
    float gap_y = fabsf(fmatrix[0].y) + fabsf(fmatrix[1].y) + fabsf(fmatrix[2].y);
    float gap_max = 2 * MAX(gap_x, gap_y);
    for (SInt max_level = 0; max_level <= ORTHO_LEVEL_LIMIT; max_level++) {
        if (gap_max < (1 << max_level)) return max_level;
    }
    return -1; // too big; can't render
}

SInt calculate_ortho_matrix(BatcherInternal* batcher, ProjectedVertex* grid, Vector3S* matrix,
    SInt* octant_order, SInt* starting_octant)
{
    Vector3F* fmatrix = (Vector3F*)matrix;
    fmatrix[0].x = GRID_AXIS_CAGE(grid, 1, 0, 0, projection.x);
    fmatrix[0].y = GRID_AXIS_CAGE(grid, 1, 0, 0, projection.y);
    fmatrix[0].z = GRID_AXIS_CAGE(grid, 1, 0, 0, position.z);
    fmatrix[1].x = GRID_AXIS_CAGE(grid, 0, 1, 0, projection.x);
    fmatrix[1].y = GRID_AXIS_CAGE(grid, 0, 1, 0, projection.y);
    fmatrix[1].z = GRID_AXIS_CAGE(grid, 0, 1, 0, position.z);
    fmatrix[2].x = GRID_AXIS_CAGE(grid, 0, 0, 1, projection.x);
    fmatrix[2].y = GRID_AXIS_CAGE(grid, 0, 0, 1, projection.y);
    fmatrix[2].z = GRID_AXIS_CAGE(grid, 0, 0, 1, position.z);
    fmatrix[3].x = GRID_CENTER_CAGE(grid, projection.x);
    fmatrix[3].y = GRID_CENTER_CAGE(grid, projection.y);
    fmatrix[3].z = GRID_CENTER_CAGE(grid, position.z);
    
    SInt max_level = calculate_max_level(fmatrix);
    if (max_level < 0) return -1;
    
    // It's easier to calculate these values using floats, to avoid overflow
    *octant_order = calculate_octant_order(fmatrix);
    *starting_octant = calculate_starting_octant_ortho(fmatrix);
    
    #ifdef DMIR_COORD_FIXED
    float level_scale = (float)(1 << (SUBPIXEL_BITS - max_level));
    matrix[0].x = (int32_t)(fmatrix[0].x * level_scale);
    matrix[0].y = (int32_t)(fmatrix[0].y * level_scale);
    matrix[1].x = (int32_t)(fmatrix[1].x * level_scale);
    matrix[1].y = (int32_t)(fmatrix[1].y * level_scale);
    matrix[2].x = (int32_t)(fmatrix[2].x * level_scale);
    matrix[2].y = (int32_t)(fmatrix[2].y * level_scale);
    matrix[3].x = (int32_t)(fmatrix[3].x * SUBPIXEL_SIZE);
    matrix[3].y = (int32_t)(fmatrix[3].y * SUBPIXEL_SIZE);
    matrix[0].x = (matrix[0].x >> 1) << max_level;
    matrix[0].y = (matrix[0].y >> 1) << max_level;
    matrix[1].x = (matrix[1].x >> 1) << max_level;
    matrix[1].y = (matrix[1].y >> 1) << max_level;
    matrix[2].x = (matrix[2].x >> 1) << max_level;
    matrix[2].y = (matrix[2].y >> 1) << max_level;
    #else
    matrix[0].x *= 0.5f;
    matrix[0].y *= 0.5f;
    matrix[1].x *= 0.5f;
    matrix[1].y *= 0.5f;
    matrix[2].x *= 0.5f;
    matrix[2].y *= 0.5f;
    #endif
    
    matrix[0].z = dz_to_depth(batcher, fmatrix[0].z);
    matrix[1].z = dz_to_depth(batcher, fmatrix[1].z);
    matrix[2].z = dz_to_depth(batcher, fmatrix[2].z);
    matrix[3].z = z_to_depth(batcher, fmatrix[3].z);
    
    #ifdef DMIR_DEPTH_INT32
    matrix[0].z >>= 1;
    matrix[1].z >>= 1;
    matrix[2].z >>= 1;
    #else
    matrix[0].z *= 0.5f;
    matrix[1].z *= 0.5f;
    matrix[2].z *= 0.5f;
    #endif
    
    return max_level;
}

void calculate_ortho_extent(Vector3S* matrix, Vector3S* extent, Effects* effects) {
    // Note: while we technically can estimate the square / circle bounds
    // in the cage-subdivision traversal, they would be wildly inaccurate
    // (and enormous) for nodes with large perspective distortion, which
    // can significantly reduce the performance.
    // Calculating the square / circle bounds in ortho may optimistically
    // cull some nodes for which the leaf bounds would actually be visible,
    // but that is a much more acceptable tradeoff.
    
    if (effects->shape == DMIR_SHAPE_CIRCLE) {
        // Diagonals are symmetric, so we only need to consider 4 corners
        float radius_squared_max = 0;
        for (SInt sy = -1; sy <= 1; sy += 2) {
            for (SInt sx = -1; sx <= 1; sx += 2) {
                float dx = matrix[0].x * sx + matrix[1].x * sy + matrix[2].x;
                float dy = matrix[0].y * sx + matrix[1].y * sy + matrix[2].y;
                float radius_squared = dx * dx + dy * dy;
                MAX_UPDATE(radius_squared_max, radius_squared);
            }
        }
        
        float radius = sqrtf(radius_squared_max) * 2;
        extent->x = (Coord)radius;
        extent->y = (Coord)radius;
    } else {
        #ifdef DMIR_COORD_FIXED
        extent->x = (abs(matrix[0].x) + abs(matrix[1].x) + abs(matrix[2].x)) * 2;
        extent->y = (abs(matrix[0].y) + abs(matrix[1].y) + abs(matrix[2].y)) * 2;
        #else
        extent->x = (fabsf(matrix[0].x) + fabsf(matrix[1].x) + fabsf(matrix[2].x)) * 2;
        extent->y = (fabsf(matrix[0].y) + fabsf(matrix[1].y) + fabsf(matrix[2].y)) * 2;
        #endif
        
        if (effects->shape == DMIR_SHAPE_SQUARE) {
            extent->x = extent->y = MAX(extent->x, extent->y);
        }
    }
    
    #ifdef DMIR_DEPTH_INT32
    extent->z = (abs(matrix[0].z) + abs(matrix[1].z) + abs(matrix[2].z)) * 2;
    #else
    extent->z = (fabsf(matrix[0].z) + fabsf(matrix[1].z) + fabsf(matrix[2].z)) * 2;
    #endif
}

Coord calculate_ortho_dilation(Effects* effects, Vector3S* extent) {
    Coord dilation_abs = (Coord)(effects->dilation_abs * SUBPIXEL_SIZE);
    Coord dilation_rel = (Coord)(effects->dilation_rel * 2 * MAX(extent->x, extent->y));
    dilation_abs += MIN(dilation_rel, DILATION_REL_SIZE_MAX * SUBPIXEL_SIZE);
    if (effects->shape == DMIR_SHAPE_CIRCLE) MAX_UPDATE(dilation_abs, 0);
    return dilation_abs;
}

void calculate_ortho_deltas(Vector3S* deltas, Vector3S* matrix, int32_t factor) {
    SInt octant = 0;
    for (SInt z = -1; z <= 1; z += 2) {
        for (SInt y = -1; y <= 1; y += 2) {
            for (SInt x = -1; x <= 1; x += 2) {
                deltas[octant].x = (matrix[0].x * x + matrix[1].x * y + matrix[2].x * z) * factor;
                deltas[octant].y = (matrix[0].y * x + matrix[1].y * y + matrix[2].y * z) * factor;
                deltas[octant].z = (matrix[0].z * x + matrix[1].z * y + matrix[2].z * z) * factor;
                octant++;
            }
        }
    }
}

typedef struct MapInfo {
    uint8_t x8[MAP_SIZE];
    uint8_t y8[MAP_SIZE];
    uint64_t x64[MAP_SIZE];
    uint64_t y64[MAP_SIZE];
    SInt shift;
    SInt half;
    SInt size8;
    SInt size36;
    SInt size64;
    uint32_t full_queue;
    uint64_t mask_bit0;
    uint64_t mask_bit1;
    uint64_t mask_bit2;
} MapInfo;

void calculate_maps(MapInfo* map, Queue* queues_forward,
    Vector3S* deltas, Vector3S extent, Coord dilation, SInt max_level)
{
    #if MAP_SIZE <= 8
    // Don't use map if it's too small or disabled
    map->size8 = -1;
    map->size36 = -1;
    map->size64 = -1;
    return;
    #else
    // Overflow can cause problems
    SInt max_dilation = (ORTHO_MAX_SIZE << SUBPIXEL_BITS) >> max_level;
    dilation = CLAMP(dilation, 0, max_dilation);
    
    // Expand the "use map" size by the dilation at pixel level
    SInt pixel_dilation_size = COORD_TO_PIXEL(dilation * 2);
    MAX_UPDATE(pixel_dilation_size, 0);
    map->size8 = MAP_2 + pixel_dilation_size;
    map->size36 = MAP_3 + pixel_dilation_size;
    map->size64 = MAX(MAP_3, MAP_4) + pixel_dilation_size;
    
    // We sample the map at one-above-pixel-size level, so
    // we need to adjust the dilation size for that level
    if (max_level > 0) dilation *= (1 << (max_level - 1));
    
    Coord max_size = (MAX(extent.x, extent.y) + dilation) * 2 + 1;
    SInt safe_size = MAP_SIZE - 2; // ensure 1-pixel empty border, just to be safe
    
    map->shift = 0;
    while ((safe_size << map->shift) < max_size) map->shift++;
    
    map->half = (MAP_SIZE << map->shift) >> 1;
    
    Coord map_center = COORD_HALVE(MAP_SIZE << map->shift);
    
    Coord extent8_x = COORD_HALVE(extent.x);
    Coord extent8_y = COORD_HALVE(extent.y);
    Coord extent64_x = COORD_HALVE(extent8_x);
    Coord extent64_y = COORD_HALVE(extent8_y);
    
    extent8_x += dilation;
    extent8_y += dilation;
    extent64_x += dilation;
    extent64_y += dilation;
    
    #ifdef DMIR_MAP_ENSURE_PIXEL
    // Imitate "pixel blitting", in the sense of
    // ensuring that each node covers a pixel
    Coord full_pixel_size = map->half >> 1;
    Coord half_pixel_size = full_pixel_size >> 1;
    MAX_UPDATE(extent8_x, full_pixel_size);
    MAX_UPDATE(extent8_y, full_pixel_size);
    MAX_UPDATE(extent64_x, half_pixel_size);
    MAX_UPDATE(extent64_y, half_pixel_size);
    #endif
    
    for (SInt i = 0; i < MAP_SIZE; i++) {
        map->x8[i] = 0;
        map->y8[i] = 0;
        map->x64[i] = 0;
        map->y64[i] = 0;
    }
    
    for (SInt octant8 = 0; octant8 < 8; octant8++) {
        Coord center8_x = map_center + deltas[octant8].x;
        Coord center8_y = map_center + deltas[octant8].y;
        
        SInt min8_x = (center8_x - extent8_x) >> map->shift;
        SInt min8_y = (center8_y - extent8_y) >> map->shift;
        SInt max8_x = (center8_x + extent8_x) >> map->shift;
        SInt max8_y = (center8_y + extent8_y) >> map->shift;
        
        // MAX_UPDATE(min8_x, 1);
        // MAX_UPDATE(min8_y, 1);
        // MIN_UPDATE(max8_x, (MAP_SIZE-2));
        // MIN_UPDATE(max8_y, (MAP_SIZE-2));
        
        uint8_t mask8 = (uint8_t)(1 << octant8);
        
        for (SInt x = min8_x; x <= max8_x; x++) {
            map->x8[x] |= mask8;
        }
        for (SInt y = min8_y; y <= max8_y; y++) {
            map->y8[y] |= mask8;
        }
        
        for (SInt octant64 = 0; octant64 < 8; octant64++) {
            Coord center64_x = center8_x + COORD_HALVE(deltas[octant64].x);
            Coord center64_y = center8_y + COORD_HALVE(deltas[octant64].y);
            
            SInt min64_x = (center64_x - extent64_x) >> map->shift;
            SInt min64_y = (center64_y - extent64_y) >> map->shift;
            SInt max64_x = (center64_x + extent64_x) >> map->shift;
            SInt max64_y = (center64_y + extent64_y) >> map->shift;
            
            // MAX_UPDATE(min64_x, 1);
            // MAX_UPDATE(min64_y, 1);
            // MIN_UPDATE(max64_x, (MAP_SIZE-2));
            // MIN_UPDATE(max64_y, (MAP_SIZE-2));
            
            uint64_t mask64 = (UINT64_C(1) << octant64) << (octant8 * 8);
            
            for (SInt x = min64_x; x <= max64_x; x++) {
                map->x64[x] |= mask64;
            }
            for (SInt y = min64_y; y <= max64_y; y++) {
                map->y64[y] |= mask64;
            }
        }
    }
    
    map->full_queue = queues_forward[255].octants;
    map->mask_bit0 = 0;
    map->mask_bit1 = 0;
    map->mask_bit2 = 0;
    for (SInt item = 0, queue = map->full_queue; item < 8; item++, queue >>= 4) {
        uint64_t octant_mask = UINT64_C(255) << ((queue & 7) * 8);
        if ((item & 1) == 0) map->mask_bit0 |= octant_mask;
        if ((item & 2) == 0) map->mask_bit1 |= octant_mask;
        if ((item & 4) == 0) map->mask_bit2 |= octant_mask;
    }
    #endif
}

typedef struct LocalVariables {
    BatcherInternal* batcher;
    RendererInternal* renderer;
    FramebufferInternal* framebuffer;
    Fragment* fragments;
    Geometry* geometry;
    Queue* queues_forward;
    MapInfo map;
    Stencil* local_stencil;
    AffineID affine_id;
    int32_t max_stack_level;
    int32_t max_level;
    int32_t shape;
    Coord dilation;
    Rect clip_rect;
    SInt offset_x;
    SInt offset_y;
    Vector3S position;
    Address node_ref;
    Address data_ref;
    Rect rect;
    SInt size_max;
    Stencil row_mask;
    uint8_t mask;
    uint8_t is_cube;
    NodeBox node_box;
    int32_t octant;
    uint32_t map_traversals_exist;
    Traversal map_traversals[9];
} LocalVariables;

typedef struct CircleParams {
    Coord radius;
    Coord radius2;
    Coord start_dx;
    Coord start_dy;
    Coord step_add;
    Coord step_add_2;
    Coord distance_2_y;
} CircleParams;

static inline void calc_circle_params(CircleParams* c,
    Vector3S* position, Vector3S* extent, SInt min_x, SInt min_y)
{
    c->radius = extent->x; // circle extent is the same for x and y
    
    #ifdef DMIR_COORD_FIXED
    const int MAGNITUDE_LIMIT = 23170; // (2 * 23170)^2 < 2^31
    SInt circle_shift = SUBPIXEL_BITS;
    while ((c->radius > MAGNITUDE_LIMIT) | ((1 << circle_shift) > MAGNITUDE_LIMIT)) {
        circle_shift--;
        c->radius >>= 1;
    }
    #endif
    
    c->radius2 = c->radius * c->radius;
    
    #ifdef DMIR_COORD_FIXED
    c->start_dx = (PIXEL_TO_COORD(min_x) - position->x) >> (SUBPIXEL_BITS - circle_shift);
    c->start_dy = (PIXEL_TO_COORD(min_y) - position->y) >> (SUBPIXEL_BITS - circle_shift);
    c->step_add = 1 << circle_shift;
    SInt step_shift = circle_shift + 1;
    c->step_add_2 = c->step_add * c->step_add;
    c->distance_2_y = c->start_dx * c->start_dx + c->start_dy * c->start_dy;
    c->start_dx <<= step_shift;
    c->start_dy <<= step_shift;
    c->step_add <<= step_shift;
    #else
    c->start_dx = PIXEL_TO_COORD(min_x) - position->x;
    c->start_dy = PIXEL_TO_COORD(min_y) - position->y;
    c->step_add = 2;
    c->step_add_2 = 1;
    c->distance_2_y = c->start_dx * c->start_dx + c->start_dy * c->start_dy;
    c->start_dx *= 2;
    c->start_dy *= 2;
    #endif
}

static void draw_circle(LocalVariables* v, OrthoStackItem* stack) {
    CircleParams c;
    calc_circle_params(&c, &v->position, &stack->extent, v->rect.min_x, v->rect.min_y);
    
    for (SInt y = v->rect.min_y; y <= v->rect.max_y; y++) {
        Coord distance_2 = c.distance_2_y, row_dx = c.start_dx;
        
        for (SInt x = v->rect.min_x; x <= v->rect.max_x; x++) {
            SInt is_inside = (distance_2 <= c.radius2);
            distance_2 += row_dx + c.step_add_2;
            row_dx += c.step_add;
            
            if (!is_inside) continue;
            
            if (v->local_stencil) {
                Stencil stencil_mask = ((Stencil)1) << x;
                if (!(v->local_stencil[y] & stencil_mask)) continue;
                v->local_stencil[y] &= ~stencil_mask;
            }
            
            SInt abs_x = x + v->offset_x;
            SInt abs_y = y + v->offset_y;
            CHECK_AND_WRITE_STENCIL(v->framebuffer, abs_x, abs_y, continue);
            SPLAT(v->framebuffer, v->fragments, abs_x, abs_y, v->position.z,
                v->affine_id, v->data_ref, continue);
        }
        
        c.distance_2_y += c.start_dy + c.step_add_2;
        c.start_dy += c.step_add;
    }
}

static inline void draw_splat(LocalVariables* v, OrthoStackItem* stack) {
    if (v->size_max == 0) {
        STAT_INCREMENT(v->framebuffer, DMIR_SPLATS_1PX);
    } else {
        STAT_INCREMENT(v->framebuffer, DMIR_SPLATS_LEAF);
        
        if (v->shape == DMIR_SHAPE_CIRCLE) {
            RECT_CLIP(v->rect, v->clip_rect);
            draw_circle(v, stack);
            return;
        } else {
            if (v->shape == DMIR_SHAPE_POINT) {
                Coord dilation = MAX(v->dilation, 0);
                RECT_FROM_POINT(v->rect, v->position, dilation, dilation);
                v->row_mask = (STENCIL_CLEAR << v->rect.min_x) ^ ((STENCIL_CLEAR << 1) << v->rect.max_x);
            }
        }
    }
    
    RECT_CLIP(v->rect, v->clip_rect);
    
    for (SInt y = v->rect.min_y; y <= v->rect.max_y; y++) {
        for (SInt x = v->rect.min_x; x <= v->rect.max_x; x++) {
            SInt abs_x = x + v->offset_x;
            SInt abs_y = y + v->offset_y;
            CHECK_AND_WRITE_STENCIL(v->framebuffer, abs_x, abs_y, continue);
            SPLAT(v->framebuffer, v->fragments, abs_x, abs_y, v->position.z,
                v->affine_id, v->data_ref, continue);
        }
    }
    
    if (v->local_stencil) {
        for (; v->rect.min_y <= v->rect.max_y; v->rect.min_y++) {
            v->local_stencil[v->rect.min_y] &= ~v->row_mask;
        }
    }
}

#ifdef USE_MAP
static void draw_map8(LocalVariables* v, OrthoStackItem* stack) {
    v->map_traversals_exist = 0;
    
    STAT_INCREMENT(v->framebuffer, DMIR_SPLATS_2PX);
    
    RECT_CLIP(v->rect, v->clip_rect);
    
    Coord min_mx = PIXEL_TO_COORD(v->rect.min_x) - (v->position.x - (v->map.half >> stack->level));
    Coord min_my = PIXEL_TO_COORD(v->rect.min_y) - (v->position.y - (v->map.half >> stack->level));
    SInt map_shift = v->map.shift - stack->level;
    
    Coord mx, my;
    SInt x, y;
    for (my = min_my, y = v->rect.min_y; y <= v->rect.max_y; y++, my += SUBPIXEL_SIZE) {
        uint8_t mask_y = v->map.y8[my >> map_shift] & v->mask;
        for (mx = min_mx, x = v->rect.min_x; x <= v->rect.max_x; x++, mx += SUBPIXEL_SIZE) {
            uint8_t mask_xy = v->map.x8[mx >> map_shift] & mask_y;
            
            if (mask_xy == 0) continue;
            
            if (v->local_stencil) {
                Stencil stencil_mask = ((Stencil)1) << x;
                if (!(v->local_stencil[y] & stencil_mask)) continue;
                v->local_stencil[y] &= ~stencil_mask;
            }
            
            SInt abs_x = x + v->offset_x;
            SInt abs_y = y + v->offset_y;
            CHECK_AND_WRITE_STENCIL(v->framebuffer, abs_x, abs_y, continue);
            
            uint32_t octant = v->queues_forward[mask_xy].octants & 7;
            Depth z = v->position.z + stack->deltas[octant].z;
            
            if (!v->is_cube) {
                if (!v->map_traversals_exist) {
                    if (!v->geometry->traverse_next(
                        v->geometry, &stack->traversal, v->octant,
                        &v->map_traversals[8])) continue;
                    v->map_traversals_exist = TRUE;
                }
                v->data_ref = v->map_traversals[8].data[octant];
            }
            
            SPLAT(v->framebuffer, v->fragments, abs_x, abs_y, z,
                v->affine_id, v->data_ref, continue);
        }
    }
}

static void draw_map64(LocalVariables* v, OrthoStackItem* stack) {
    v->map_traversals_exist = 0;
    
    SInt use_suboctants = (v->size_max >= v->map.size36) & (stack->level != (v->max_level-1));
    
    if (use_suboctants) {
        STAT_INCREMENT(v->framebuffer, DMIR_SPLATS_4PX);
    } else {
        STAT_INCREMENT(v->framebuffer, DMIR_SPLATS_3PX);
    }
    
    RECT_CLIP(v->rect, v->clip_rect);
    
    Coord min_mx = PIXEL_TO_COORD(v->rect.min_x) - (v->position.x - (v->map.half >> stack->level));
    Coord min_my = PIXEL_TO_COORD(v->rect.min_y) - (v->position.y - (v->map.half >> stack->level));
    SInt map_shift = v->map.shift - stack->level;
    
    uint64_t mask64 = 0;
    if (!v->is_cube) {
        Queue queue = v->queues_forward[v->mask];
        
        if (!v->geometry->traverse_next(
            v->geometry, &stack->traversal, v->octant,
            &v->map_traversals[8])) return;
        
        for (; queue.octants != 0; queue.octants >>= 4) {
            uint64_t mask8 = v->map_traversals[8].mask[queue.octants & 7];
            mask64 |= (mask8 ? mask8 : 255) << ((queue.octants & 7) * 8);
        }
    } else {
        mask64 = UINT64_MAX;
    }
    
    Coord mx, my;
    SInt x, y;
    for (my = min_my, y = v->rect.min_y; y <= v->rect.max_y; y++, my += SUBPIXEL_SIZE) {
        uint64_t mask_y = v->map.y64[my >> map_shift] & mask64;
        for (mx = min_mx, x = v->rect.min_x; x <= v->rect.max_x; x++, mx += SUBPIXEL_SIZE) {
            uint64_t mask_xy = v->map.x64[mx >> map_shift] & mask_y;
            
            if (mask_xy == 0) continue;
            
            if (v->local_stencil) {
                Stencil stencil_mask = ((Stencil)1) << x;
                if (!(v->local_stencil[y] & stencil_mask)) continue;
                v->local_stencil[y] &= ~stencil_mask;
            }
            
            SInt abs_x = x + v->offset_x;
            SInt abs_y = y + v->offset_y;
            CHECK_AND_WRITE_STENCIL(v->framebuffer, abs_x, abs_y, continue);
            
            SInt bit2 = ((mask_xy & v->map.mask_bit2) == 0) & 1;
            mask_xy &= v->map.mask_bit2 ^ (-(int64_t)bit2);
            SInt bit1 = ((mask_xy & v->map.mask_bit1) == 0) & 1;
            mask_xy &= v->map.mask_bit1 ^ (-(int64_t)bit1);
            SInt bit0 = ((mask_xy & v->map.mask_bit0) == 0) & 1;
            SInt queue_item = bit0 | (bit1 << 1) | (bit2 << 2);
            SInt octant64 = (v->map.full_queue >> (queue_item << 2)) & 7;
            
            Depth z = v->position.z + stack->deltas[octant64].z;
            
            if (!v->is_cube) {
                v->data_ref = v->map_traversals[8].data[octant64];
                
                if (use_suboctants) {
                    uint8_t sub_mask_full = v->map_traversals[8].mask[octant64];
                    if (sub_mask_full != 0) {
                        if ((v->map_traversals_exist & (1 << octant64)) == 0) {
                            if (!v->geometry->traverse_next(
                                v->geometry, &v->map_traversals[8], octant64,
                                &v->map_traversals[octant64])) continue;
                            v->map_traversals_exist |= (1 << octant64);
                        }
                        uint8_t sub_mask = sub_mask_full & (mask_xy >> (octant64 * 8));
                        uint32_t octant = v->queues_forward[sub_mask].octants & 7;
                        z += DEPTH_HALVE(stack->deltas[octant].z);
                        v->data_ref = v->map_traversals[octant64].data[octant];
                    }
                }
            }
            
            SPLAT(v->framebuffer, v->fragments, abs_x, abs_y, z,
                v->affine_id, v->data_ref, continue);
        }
    }
}
#endif

#ifdef STENCIL_LOCAL_SIZE
static void initialize_local_stencil(FramebufferInternal* framebuffer,
    OrthoStackItem* stack, Stencil* local_stencil, Rect* rect, Depth depth)
{
    SInt min_x = MAX(rect->min_x, stack->rect.min_x);
    SInt min_y = MAX(rect->min_y, stack->rect.min_y);
    SInt max_x = MIN(rect->max_x, stack->rect.max_x);
    SInt max_y = MIN(rect->max_y, stack->rect.max_y);
    
    #if STENCIL_SHIFT_Y == 0
    SInt min_tx = min_x & ~STENCIL_MASK_X;
    SInt max_tx = max_x & ~STENCIL_MASK_X;
    SInt tile_col = min_tx >> STENCIL_SHIFT_X;
    SInt tile_row = rect->min_y;
    Stencil* stencil_x = framebuffer->stencil_x;
    
    SInt local_ymin = stack->rect.min_y - rect->min_y;
    SInt local_ymax = stack->rect.max_y - rect->min_y;
    
    for (SInt y = 0; y < STENCIL_LOCAL_SIZE; y++, tile_row++) {
        local_stencil[y] = 0;
        
        if ((y < local_ymin) | (y > local_ymax)) continue;
        
        StencilTile* tile = framebuffer->stencil_tiles +
            (tile_col + (tile_row << framebuffer->stencil_row_shift));
        for (SInt tx = min_tx; tx <= max_tx; tx += STENCIL_SIZE_X, tile++) {
            Stencil pixel_mask = tile->self;
            pixel_mask &= (stencil_x[rect->min_x - tx] ^ stencil_x[rect->max_x - tx + 1]);
            if (depth >= tile->depth) pixel_mask &= tile->scene;
            
            if (tx < rect->min_x) {
                local_stencil[y] |= pixel_mask >> (rect->min_x - tx);
            } else {
                local_stencil[y] |= pixel_mask << (tx - rect->min_x);
            }
        }
    }
    #else
    SInt min_tx = min_x & ~STENCIL_MASK_X;
    SInt max_tx = max_x & ~STENCIL_MASK_X;
    SInt min_ty = min_y & ~STENCIL_MASK_Y;
    SInt max_ty = max_y & ~STENCIL_MASK_Y;
    SInt tile_col = min_tx >> STENCIL_SHIFT_X;
    SInt tile_row = min_ty >> STENCIL_SHIFT_Y;
    Stencil* stencil_x = framebuffer->stencil_x;
    Stencil* stencil_y = framebuffer->stencil_y;
    
    SInt local_ymin = stack->rect.min_y - rect->min_y;
    SInt local_ymax = stack->rect.max_y - rect->min_y;
    
    for (SInt i = 0; i < STENCIL_LOCAL_SIZE; i++) {
        local_stencil[i] = 0;
    }
    
    for (SInt ty = min_ty; ty <= max_ty; ty += STENCIL_SIZE_Y, tile_row++) {
        StencilTile* tile = framebuffer->stencil_tiles +
            (tile_col + (tile_row << framebuffer->stencil_row_shift));
        
        SInt tile_ymin = ty - rect->min_y;
        SInt tile_ymax = tile_ymin + (STENCIL_SIZE_Y - 1);
        MAX_UPDATE(tile_ymin, local_ymin);
        MIN_UPDATE(tile_ymax, local_ymax);
        
        for (SInt tx = min_tx; tx <= max_tx; tx += STENCIL_SIZE_X, tile++) {
            Stencil pixel_mask = tile->self;
            pixel_mask &=
                (stencil_x[rect->min_x - tx] ^ stencil_x[rect->max_x - tx + 1]) &
                (stencil_y[rect->min_y - ty] ^ stencil_y[rect->max_y - ty + 1]);
            if (depth >= tile->depth) pixel_mask &= tile->scene;
            
            SInt offset_x = rect->min_x - tx;
            SInt offset_y = rect->min_y - ty;
            
            for (SInt y = tile_ymin; y <= tile_ymax; y++) {
                Stencil row_mask = pixel_mask >> ((y + offset_y) * STENCIL_SIZE_X);
                if (offset_x > 0) {
                    local_stencil[y] |= row_mask >> offset_x;
                } else {
                    local_stencil[y] |= row_mask << -offset_x;
                }
            }
        }
    }
    #endif
}

static void initialize_ortho_local(LocalVariables* v, OrthoStackItem* stack) {
    Coord extent_x = (stack-1)->extent.x + SUBPIXEL_SIZE;
    Coord extent_y = (stack-1)->extent.y + SUBPIXEL_SIZE;
    Depth depth = v->position.z - (stack-1)->extent.z;
    
    Rect local_rect;
    RECT_FROM_POINT(local_rect, v->position, extent_x, extent_y);
    
    v->offset_x = local_rect.min_x;
    v->offset_y = local_rect.min_y;
    
    v->clip_rect = (Rect){
        .min_x = stack->rect.min_x - v->offset_x,
        .min_y = stack->rect.min_y - v->offset_y,
        .max_x = stack->rect.max_x - v->offset_x,
        .max_y = stack->rect.max_y - v->offset_y,
    };
    
    initialize_local_stencil(v->framebuffer, stack, v->local_stencil, &local_rect, depth);
    
    stack->center.x -= SUBPIXEL_SIZE * v->offset_x;
    stack->center.y -= SUBPIXEL_SIZE * v->offset_y;
    stack->rect.min_x -= v->offset_x;
    stack->rect.min_y -= v->offset_y;
    stack->rect.max_x -= v->offset_x;
    stack->rect.max_y -= v->offset_y;
}
#endif

Bool initialize_ortho(LocalVariables* v, OrthoStackItem* stack, ProjectedVertex* grid, Effects* effects) {
    Vector3S matrix[4];
    SInt octant_order, starting_octant;
    SInt max_level = calculate_ortho_matrix(v->batcher, grid, matrix, &octant_order, &starting_octant);
    if (max_level < 0) return FALSE;
    
    v->max_level = (effects->max_level < 0 ? max_level : MIN(effects->max_level, max_level));
    
    // In the cube case, we need to calculate extents/deltas to the pixel level
    v->max_stack_level = (v->shape != DMIR_SHAPE_CUBE) ? v->max_level : max_level;
    
    v->queues_forward = lookups.sparse + (((octant_order << 3) | (starting_octant ^ 0b000)) << 8);
    
    calculate_ortho_extent(matrix, &stack->extent, effects);
    
    v->dilation = calculate_ortho_dilation(effects, &stack->extent);
    
    calculate_ortho_deltas(stack->deltas, matrix, 2);
    
    for (SInt level = 1; level <= v->max_stack_level; level++) {
        stack[level].extent.x = COORD_HALVE(stack[level-1].extent.x);
        stack[level].extent.y = COORD_HALVE(stack[level-1].extent.y);
        stack[level].extent.z = DEPTH_HALVE(stack[level-1].extent.z);
        for (SInt octant = 0; octant < 8; octant++) {
            stack[level].deltas[octant].x = COORD_HALVE(stack[level-1].deltas[octant].x);
            stack[level].deltas[octant].y = COORD_HALVE(stack[level-1].deltas[octant].y);
            stack[level].deltas[octant].z = DEPTH_HALVE(stack[level-1].deltas[octant].z);
        }
    }
    
    // Calculate maps before we dilate the extent
    if (v->max_stack_level > 0) {
        calculate_maps(&v->map, v->queues_forward, stack[1].deltas, stack->extent, v->dilation, max_level);
    }
    
    Depth min_depth = z_to_depth(v->batcher, v->batcher->frustum_bounds.min_z);
    Depth max_depth = z_to_depth(v->batcher, v->batcher->frustum_bounds.max_z);
    for (SInt level = 0; level <= v->max_stack_level; level++) {
        stack[level].level = level;
        stack[level].extent.x += v->dilation;
        stack[level].extent.y += v->dilation;
        stack[level].min_z = min_depth - stack[level].extent.z;
        stack[level].max_z = max_depth + stack[level].extent.z;
    }
    
    v->position = (Vector3S){.x = matrix[3].x, .y = matrix[3].y, .z = matrix[3].z};
    
    stack->rect = v->renderer->api.rect;
    
    stack->is_cube = v->is_cube;
    
    if (v->geometry->evaluate != NULL) {
        stack->traversal.mask[0] = 255;
        stack->traversal.node[0] = v->node_ref;
        stack->traversal.data[0] = v->data_ref;
    } else {
        if (!v->geometry->traverse_start(v->geometry, v->node_ref, v->data_ref, &stack->traversal)) return FALSE;
    }
    
    stack->queue = 0;
    
    v->local_stencil = NULL;
    v->offset_x = 0;
    v->offset_y = 0;
    v->clip_rect = stack->rect;
    
    return TRUE;
}

#define CONCAT(x, y) x ## _ ## y
#define RENDER_ORTHO(mode) CONCAT(render_ortho, mode)

#define MODE tree
#define MODE_TREE
#include "render_ortho.c"
#undef MODE
#undef MODE_TREE

#define MODE eval
#define MODE_EVAL
#include "render_ortho.c"
#undef MODE
#undef MODE_EVAL

///////////////////////////////////////////
// Public API and some related functions //
///////////////////////////////////////////

#ifdef DMIR_ROW_POW2
int32_t dmir_row_size(DMirFramebuffer* framebuffer) {
    return 1 << framebuffer->row_shift;
}
int32_t dmir_pixel_index(DMirFramebuffer* framebuffer, int32_t x, int32_t y) {
    return x + (y << framebuffer->row_shift);
}
#else
int32_t dmir_row_size(DMirFramebuffer* framebuffer) {
    return framebuffer->size_x;
}
int32_t dmir_pixel_index(DMirFramebuffer* framebuffer, int32_t x, int32_t y) {
    return x + (y * framebuffer->size_x);
}
#endif

void dmir_lookups_initialize(void) {
    lookups_initialize();
}

Framebuffer* dmir_framebuffer_make(uint32_t size_x, uint32_t size_y) {
    FramebufferInternal* framebuffer = malloc(sizeof(FramebufferInternal));
    
    if (framebuffer) {
        framebuffer->api.depth = NULL;
        framebuffer->api.voxel = NULL;
        
        framebuffer->stencil_tiles = NULL;
        framebuffer->stencil_x_base = NULL;
        framebuffer->stencil_y_base = NULL;
        framebuffer->stencil_x = NULL;
        framebuffer->stencil_y = NULL;
        
        dmir_framebuffer_resize((Framebuffer*)framebuffer, size_x, size_y);
    }
    
    return (Framebuffer*)framebuffer;
}

void framebuffer_free_channels(FramebufferInternal* framebuffer) {
    if (framebuffer->api.depth) free(framebuffer->api.depth);
    if (framebuffer->api.voxel) free(framebuffer->api.voxel);
    if (framebuffer->stencil_tiles) free(framebuffer->stencil_tiles);
    if (framebuffer->stencil_x_base) free(framebuffer->stencil_x_base);
    if (framebuffer->stencil_y_base) free(framebuffer->stencil_y_base);
}

void dmir_framebuffer_free(Framebuffer* framebuffer_ptr) {
    FramebufferInternal* framebuffer = (FramebufferInternal*)framebuffer_ptr;
    
    framebuffer_free_channels(framebuffer);
    
    free(framebuffer);
}

Stencil* stencil_masks_make(SInt size, SInt margin, Stencil mask, SInt step) {
    SInt count = size + 1 + margin * 2;
    Stencil* result = malloc(count * sizeof(Stencil));
    for (SInt i = 0; i <= margin; i++) {
        result[i] = 0;
    }
    for (SInt i = 1; i < size; i++) {
        result[margin + i] = result[margin + i - 1] | (mask << ((i-1) * step));
    }
    for (SInt i = margin + size; i < count; i++) {
        result[i] = STENCIL_CLEAR;
    }
    return result;
}

void dmir_framebuffer_resize(Framebuffer* framebuffer_ptr, uint32_t size_x, uint32_t size_y) {
    FramebufferInternal* framebuffer = (FramebufferInternal*)framebuffer_ptr;
    
    framebuffer_free_channels(framebuffer);
    
    framebuffer->api.size_x = size_x;
    framebuffer->api.size_y = size_y;
    framebuffer->api.row_shift = pow2_ceil(size_x);
    
    SInt buffer_size = dmir_row_size((Framebuffer*)framebuffer) * framebuffer->api.size_y;
    buffer_size = MAX(buffer_size, 1); // safeguard if width or height is 0
    framebuffer->api.depth = malloc(buffer_size * sizeof(Depth));
    framebuffer->api.voxel = malloc(buffer_size * sizeof(VoxelRef));
    
    #if STENCIL_BITS > 0
    framebuffer->api.stencil_size_x = STENCIL_SIZE_X;
    framebuffer->api.stencil_size_y = STENCIL_SIZE_Y;
    framebuffer->api.stencil_count_x = (size_x + (STENCIL_SIZE_X - 1)) >> STENCIL_SHIFT_X;
    framebuffer->api.stencil_count_y = (size_y + (STENCIL_SIZE_Y - 1)) >> STENCIL_SHIFT_Y;
    framebuffer->stencil_row_shift = pow2_ceil(framebuffer->api.stencil_count_x);
    #else
    framebuffer->api.stencil_size_x = 1;
    framebuffer->api.stencil_size_y = 1;
    framebuffer->api.stencil_count_x = 0;
    framebuffer->api.stencil_count_y = 0;
    framebuffer->stencil_row_shift = 0;
    #endif
    
    SInt tiles_count = (1 << framebuffer->stencil_row_shift) * framebuffer->api.stencil_count_y;
    framebuffer->stencil_tiles = malloc(tiles_count * sizeof(StencilTile));
    
    // Add 1 just to be safe
    SInt margin_x = size_x + 1;
    SInt margin_y = size_y + 1;
    framebuffer->stencil_x_base = stencil_masks_make(
        STENCIL_SIZE_X, margin_x, STENCIL_LUT_MASK_X, STENCIL_LUT_STEP_X);
    framebuffer->stencil_y_base = stencil_masks_make(
        STENCIL_SIZE_Y, margin_y, STENCIL_LUT_MASK_Y, STENCIL_LUT_STEP_Y);
    framebuffer->stencil_x = framebuffer->stencil_x_base + margin_x;
    framebuffer->stencil_y = framebuffer->stencil_y_base + margin_y;
    
    dmir_framebuffer_clear((Framebuffer*)framebuffer);
}

void dmir_framebuffer_clear(Framebuffer* framebuffer_ptr) {
    FramebufferInternal* framebuffer = (FramebufferInternal*)framebuffer_ptr;
    
    VoxelRef voxel = {.affine_id = -1, .voxel_id = 0};
    
    for (SInt y = 0; y < framebuffer->api.size_y; y++) {
        Depth* depth_row = framebuffer->api.depth + PIXEL_INDEX(framebuffer, 0, y);
        VoxelRef* voxel_row = framebuffer->api.voxel + PIXEL_INDEX(framebuffer, 0, y);
        for (SInt x = 0; x < framebuffer->api.size_x; x++) {
            depth_row[x] = DMIR_MAX_DEPTH;
            voxel_row[x] = voxel;
        }
    }
    
    StencilTile* stencil_row = framebuffer->stencil_tiles;
    SInt stencil_row_step = 1 << framebuffer->stencil_row_shift;
    for (SInt ty = 0; ty < framebuffer->api.stencil_count_y; ty++) {
        for (SInt tx = 0; tx < framebuffer->api.stencil_count_x; tx++) {
            stencil_row[tx].self = STENCIL_CLEAR;
            stencil_row[tx].scene = STENCIL_CLEAR;
            stencil_row[tx].depth = DMIR_MAX_DEPTH;
            stencil_row[tx].update = 0;
        }
        stencil_row += stencil_row_step;
    }
    
    for (SInt stat_id = 0; stat_id < DMIR_STATS_COUNT; stat_id++) {
        framebuffer->api.stats[stat_id] = 0;
    }
}

Bool dmir_is_occluded_quad(Framebuffer* framebuffer_ptr, Rect rect, Depth depth) {
    FramebufferInternal* framebuffer = (FramebufferInternal*)framebuffer_ptr;
    
    return (Bool)is_occluded_quad(framebuffer, &rect, depth);
}

Batcher* dmir_batcher_make() {
    BatcherInternal* batcher = malloc(sizeof(BatcherInternal));
    
    if (batcher) {
        batcher->affine_size = 1024;
        batcher->affine = malloc(batcher->affine_size * sizeof(AffineInfo));
        
        batcher->subtrees_size = 2048;
        batcher->subtrees = malloc(batcher->subtrees_size * sizeof(Subtree));
        
        batcher->stack = malloc(MAX_STACK_DEPTH * sizeof(GridStackItem));
        
        batcher->api.split_factor = INFINITY;
        batcher->api.affine_distortion = 1;
        batcher->api.ortho_distortion = 1;
        
        dmir_batcher_reset((Batcher*)batcher, DMIR_RECT_EMPTY, DMIR_FRUSTUM_DEFAULT);
    }
    
    return (Batcher*)batcher;
}

void dmir_batcher_free(Batcher* batcher_ptr) {
    BatcherInternal* batcher = (BatcherInternal*)batcher_ptr;
    
    if (batcher->affine) free(batcher->affine);
    
    if (batcher->subtrees) free(batcher->subtrees);
    
    if (batcher->stack) free(batcher->stack);
    
    free(batcher);
}

void dmir_batcher_reset(Batcher* batcher_ptr, Rect viewport, Frustum frustum) {
    BatcherInternal* batcher = (BatcherInternal*)batcher_ptr;
    
    batcher->affine_count = 0;
    batcher->subtrees_count = 0;
    batcher->sorted_head = -1;
    
    batcher->api.viewport = viewport;
    batcher->api.frustum = frustum;
    
    batcher->api.rect = DMIR_RECT_EMPTY;
    
    #ifdef DMIR_DEPTH_INT32
    batcher->depth_factor = DMIR_MAX_DEPTH / (frustum.max_depth - frustum.min_depth);
    #else
    batcher->depth_factor = 1.0f;
    #endif
    
    SInt viewport_size_x = 0;
    SInt viewport_size_y = 0;
    
    if ((viewport.max_x >= viewport.min_x) & (viewport.max_y >= viewport.min_y)) {
        viewport_size_x = viewport.max_x - viewport.min_x + 1;
        viewport_size_y = viewport.max_y - viewport.min_y + 1;
        
        float half_x = viewport_size_x * 0.5f;
        float half_y = viewport_size_y * 0.5f;
        
        batcher->scale_xy = ((half_x > half_y) ? half_x : half_y) / frustum.focal_extent;
        batcher->scale_c = 1 - frustum.perspective;
        batcher->scale_z = frustum.perspective / frustum.focal_depth;
        
        batcher->offset_x = viewport.min_x + half_x + frustum.offset_x;
        batcher->offset_y = viewport.min_y + half_y + frustum.offset_y;
        
        batcher->eye_z = -batcher->scale_c / batcher->scale_z;
        batcher->is_perspective = (batcher->scale_z > 1e-16f) | (batcher->scale_z < -1e-16f);
        batcher->clamp_z = batcher->eye_z + MAX(-batcher->eye_z, 1) * 1e-8f;
        
        batcher->frustum_bounds.min_x = viewport.min_x;
        batcher->frustum_bounds.max_x = viewport.max_x + 1;
        batcher->frustum_bounds.min_y = viewport.min_y;
        batcher->frustum_bounds.max_y = viewport.max_y + 1;
        batcher->frustum_bounds.min_z = MAX(frustum.min_depth, batcher->eye_z);
        batcher->frustum_bounds.max_z = frustum.max_depth;
    } else {
        batcher->scale_xy = 0;
        batcher->scale_c = 1;
        batcher->scale_z = 0;
        
        batcher->offset_x = 0;
        batcher->offset_y = 1;
        
        batcher->eye_z = 0;
        batcher->is_perspective = FALSE;
        batcher->clamp_z = 0;
        
        batcher->frustum_bounds.min_x = 0;
        batcher->frustum_bounds.max_x = -1;
        batcher->frustum_bounds.min_y = 0;
        batcher->frustum_bounds.max_y = -1;
        batcher->frustum_bounds.min_z = frustum.min_depth;
        batcher->frustum_bounds.max_z = frustum.max_depth;
    }
}

SInt batcher_add_affine(BatcherInternal* batcher,
    uint32_t group, Geometry* geometry, ProjectedVertex* grid)
{
    if (batcher->affine_count == batcher->affine_size) {
        batcher->affine_size *= 2;
        batcher->affine = realloc(batcher->affine, batcher->affine_size * sizeof(AffineInfo));
    }
    AffineID affine_id = batcher->affine_count;
    AffineInfo* affine_info = batcher->affine + affine_id;
    affine_info->group = group;
    affine_info->geometry = geometry;
    Vector3F* matrix_normal = (Vector3F*)(&affine_info->matrix_normal);
    matrix_normal[0] = normalized_matrix_axis(grid, 1, 0, 0);
    matrix_normal[1] = normalized_matrix_axis(grid, 0, 1, 0);
    matrix_normal[2] = normalized_matrix_axis(grid, 0, 0, 1);
    batcher->affine_count++;
    return affine_id;
}

Subtree* batcher_add_subtree(BatcherInternal* batcher) {
    if (batcher->subtrees_count == batcher->subtrees_size) {
        batcher->subtrees_size *= 2;
        batcher->subtrees = realloc(batcher->subtrees, batcher->subtrees_size * sizeof(Subtree));
    }
    
    Subtree* subtree = batcher->subtrees + batcher->subtrees_count;
    subtree->next = -1;
    batcher->subtrees_count++;
    return subtree;
}

void dmir_batcher_add(Batcher* batcher_ptr, Framebuffer* framebuffer_ptr,
    uint32_t group, float* cage_ptr, Geometry* geometry, Address root, Effects effects)
{
    BatcherInternal* batcher = (BatcherInternal*)batcher_ptr;
    FramebufferInternal* framebuffer = (FramebufferInternal*)framebuffer_ptr;
    
    GridStackItem* stack_start = batcher->stack;
    GridStackItem* stack = stack_start;
    
    initialize_grid(stack->grid, (Vector3F*)cage_ptr, batcher);
    
    float max_scale;
    Bounds bounds;
    calculate_screen_bounds(stack->grid, &bounds, &max_scale);
    
    if (!isfinite(bounds.max_z - bounds.min_z)) return;
    
    if ((effects.max_level < 0) | (effects.max_level > DMIR_LEVEL_MAX)) {
        effects.max_level = DMIR_LEVEL_MAX;
    }
    if (geometry->max_level >= 0) {
        effects.max_level = MIN(effects.max_level, geometry->max_level);
    }
    
    effects.dilation_abs = MAX(effects.dilation_abs, 0) - 0.5f;
    effects.dilation_rel = CLAMP(effects.dilation_rel, 0, 1) * 0.5f;
    
    Rect viewport = batcher->api.viewport;
    Bounds frustum_bounds = batcher->frustum_bounds;
    
    SInt viewport_size_x = viewport.max_x - viewport.min_x + 1;
    SInt viewport_size_y = viewport.max_y - viewport.min_y + 1;
    SInt viewport_size_max = MAX(viewport_size_x, viewport_size_y);
    SInt max_subtree_size = (SInt)CLAMP(batcher->api.split_factor * viewport_size_max, 0, INT32_MAX);
    MAX_UPDATE(max_subtree_size, (SInt)(effects.dilation_abs * 2 + 1));
    
    stack->level = 0;
    stack->affine_id = -1;
    stack->rect = viewport;
    stack->is_behind = (bounds.min_z <= batcher->eye_z);
    
    if (geometry->evaluate != NULL) {
        stack->traversal.mask[0] = 255;
        stack->traversal.node[0] = root;
        stack->traversal.data[0] = 0;
    } else {
        if (!geometry->traverse_start(geometry, root, 0, &stack->traversal)) return;
    }
    
    stack->queue = 0;
    
    Address node_ref = root;
    SInt octant = 0;
    SInt mask = stack->traversal.mask[octant];
    
    NodeBox node_box = {.x = 0, .y = 0, .z = 0, .level = 0};
    
    goto skip_initialization;
    
    do {
        if (stack->queue == 0) {
            stack--;
            continue;
        }
        octant = stack->queue & 7;
        stack->queue >>= 4;
        mask = stack->traversal.mask[octant];
        node_ref = stack->traversal.node[octant];
        node_box_child(&stack->node_box, octant, &node_box);
        initialize_subgrid(stack->grid, (stack-1)->grid, octant);
        calculate_screen_bounds(stack->grid, &bounds, &max_scale);
        
        skip_initialization:;
        
        STAT_INCREMENT(framebuffer, DMIR_NODES_BATCHED);
        
        if (bounds.min_z >= frustum_bounds.max_z) continue;
        
        SInt intersects_near_plane = (bounds.min_z < frustum_bounds.min_z);
        
        if (intersects_near_plane) {
            if (bounds.max_z <= frustum_bounds.min_z) continue;
            if ((!mask) | (stack->level == effects.max_level)) continue;
        }
        
        float dilation = effects.dilation_abs;
        
        SInt intersects_eye_plane = (bounds.min_z <= batcher->eye_z);
        
        if (!intersects_near_plane & (effects.dilation_rel > 0)) {
            float bounds_size_x = bounds.max_x - bounds.min_x;
            float bounds_size_y = bounds.max_y - bounds.min_y;
            float bounds_size_max = MAX(bounds_size_x, bounds_size_y);
            float dilation_rel_size = effects.dilation_rel * bounds_size_max * (1 << stack->level);
            dilation += MIN(dilation_rel_size, DILATION_REL_SIZE_MAX);
        }
        
        if (stack->is_behind) enlarge_bounds(&bounds);
        
        // Calculate screen-space bounds (in pixels)
        // (clamps to a sane range if values are too big)
        Rect rect;
        RECT_FROM_BOUNDS(rect, bounds, dilation);
        
        // Calculate node size (in pixels)
        SInt size_x = rect.max_x - rect.min_x;
        SInt size_y = rect.max_y - rect.min_y;
        SInt size_max = MAX(size_x, size_y);
        SInt is_pixel = (size_max == 0);
        SInt is_splat = (!mask) | (stack->level == effects.max_level);
        
        // Clamp to viewport
        if (stack->is_behind) {
            RECT_CLIP(rect, viewport);
        } else {
            RECT_CLIP(rect, stack->rect);
        }
        
        // Skip if not visible
        if ((rect.min_x > rect.max_x) | (rect.min_y > rect.max_y)) continue;
        
        if (!intersects_near_plane) {
            Depth depth = z_to_depth(batcher, bounds.min_z);
            
            // Do an occlusion check
            if (is_occluded_quad(framebuffer, &rect, depth)) continue;
        }
        
        if (geometry->evaluate != NULL) {
            int32_t eval_result = geometry->evaluate(geometry, &node_box, &stack->traversal.data[octant]);
            if (eval_result < 0) continue;
            is_splat |= (eval_result == 0);
            mask = 255;
        }
        
        // Calculate the remaining (non-corner) vertices
        // This is also required for calculating matrix axes and starting octant
        calculate_midpoints(stack->grid, batcher);
        
        if (stack->affine_id < 0) {
            SInt is_affine = is_pixel | is_splat;
            
            if (!is_affine) {
                float affine_distortion = calculate_affine_distortion(stack->grid);
                is_affine |= affine_distortion * max_scale < batcher->api.affine_distortion;
            }
            
            if (is_affine) {
                stack->affine_id = batcher_add_affine(batcher, group, geometry, stack->grid);
            }
        }
        
        if (stack->affine_id >= 0) {
            // Making sure that all subtrees are orthogonal would be convenient,
            // but it might result in extra nodes that would have to be sorted
            // and rendered, while they could have probably been occlusion-culled.
            // On the other hand, even if a node is orthogonal, we don't want it
            // to be too big; besides being necessary for fixed-point arithmetic,
            // we also want to take advantage of pre-sorting to reduce overdraw.
            
            if (is_pixel | is_splat | (size_max < max_subtree_size)) {
                Subtree* subtree = batcher_add_subtree(batcher);
                subtree->affine_id = stack->affine_id;
                subtree->bounds = bounds;
                subtree->rect = rect;
                cage_from_grid(stack->grid, subtree->cage);
                subtree->node_box = node_box;
                subtree->node_ref = node_ref;
                subtree->data_ref = stack->traversal.data[octant];
                subtree->effects = effects;
                subtree->effects.max_level -= stack->level;
                subtree->effects.dilation_rel *= (1 << stack->level);
                
                RECT_EXPAND(batcher->api.rect, rect);
                continue;
            }
        }
        
        // Calculate the starting octant
        SInt starting_octant;
        if (batcher->is_perspective & (bounds.min_z < batcher->eye_z)) {
            starting_octant = calculate_starting_octant_perspective(stack->grid, batcher->eye_z);
        } else {
            starting_octant = calculate_starting_octant(stack->grid);
        }
        
        SInt octant_order = calculate_octant_order_grid(stack->grid);
        
        stack++;
        stack->level = (stack-1)->level + 1;
        stack->rect = rect;
        stack->node_box = node_box;
        stack->is_behind = intersects_eye_plane;
        stack->affine_id = (stack-1)->affine_id;
        
        if (geometry->evaluate != NULL) {
            Address data_ref = (stack-1)->traversal.data[octant];
            for (SInt i = 0; i < 8; i++) {
                stack->traversal.mask[i] = 255;
                stack->traversal.node[i] = node_ref;
                stack->traversal.data[i] = data_ref;
            }
        } else if (!geometry->traverse_next(geometry, &(stack-1)->traversal, octant, &stack->traversal)) {
            stack--;
            continue;
        }
        
        Queue* queues_forward = lookups.sparse + (((octant_order << 3) | (starting_octant)) << 8);
        stack->queue = queues_forward[mask].octants;
    } while (stack > stack_start);
}

void dmir_batcher_sort(Batcher* batcher_ptr) {
    BatcherInternal* batcher = (BatcherInternal*)batcher_ptr;
    
    batcher->sorted_head = -1; // in case no subtrees were added in this pass
    
    if (batcher->subtrees_count == 0) return;
    
    Subtree* subtrees = batcher->subtrees;
    
    for (SInt index = 1; index < batcher->subtrees_count; index++) {
        subtrees[index-1].next = index;
    }
    subtrees[batcher->subtrees_count-1].next = -1;
    
    SInt head = 0;
    SInt tail = batcher->subtrees_count - 1;
    
    // Inspired by https://www.chiark.greenend.org.uk/~sgtatham/algorithms/listsort.html
    for (SInt k = 1, merges = 0; merges != 1; k *= 2) {
        merges = 0;
        SInt p = head;
        head = -1;
        tail = -1;
        
        while (p >= 0) {
            SInt psize = 0;
            SInt qsize = k;
            SInt q = p;
            while ((q >= 0) & (psize != k)) {
                psize++;
                q = subtrees[q].next;
            }
            
            while ((psize > 0) | ((qsize > 0) & (q >= 0))) {
                SInt use_p;
                if ((psize > 0) & (qsize > 0) & (q >= 0)) {
                    use_p = (subtrees[p].bounds.min_z <= subtrees[q].bounds.min_z);
                } else {
                    use_p = (psize > 0);
                }
                
                SInt e;
                if (use_p) {
                    e = p;
                    p = subtrees[p].next;
                    psize--;
                } else {
                    e = q;
                    q = subtrees[q].next;
                    qsize--;
                }
                
                if (head < 0) {
                    head = e;
                } else {
                    subtrees[tail].next = e;
                }
                tail = e;
                subtrees[tail].next = -1;
            }
            
            merges++;
            p = q;
        }
    }
    
    batcher->sorted_head = head;
    
    batcher->subtrees_count = 0;
}

void dmir_batcher_affine_get(Batcher* batcher_ptr, AffineInfo** affine_infos, uint32_t* count) {
    BatcherInternal* batcher = (BatcherInternal*)batcher_ptr;
    *affine_infos = batcher->affine;
    *count = batcher->affine_count;
}

Depth dmir_z_to_depth(Batcher* batcher_ptr, float z) {
    BatcherInternal* batcher = (BatcherInternal*)batcher_ptr;
    #ifdef DMIR_DEPTH_INT32
    z = CLAMP(z, batcher->api.frustum.min_depth, batcher->api.frustum.max_depth);
    #endif
    return z_to_depth(batcher, z);
}

Renderer* dmir_renderer_make() {
    RendererInternal* renderer = malloc(sizeof(RendererInternal));
    
    if (renderer) {
        renderer->stack = malloc(MAX_STACK_DEPTH * sizeof(GridStackItem));
        
        renderer->fragments_size = 0;
        renderer->fragments = NULL;
    }
    
    return (Renderer*)renderer;
}

void dmir_renderer_free(Renderer* renderer_ptr) {
    RendererInternal* renderer = (RendererInternal*)renderer_ptr;
    
    free(renderer->stack);
    
    if (renderer->fragments) free(renderer->fragments);
    
    free(renderer);
}

static inline void tile_depth_update(FramebufferInternal* framebuffer,
    StencilTile* tile, SInt tx, SInt ty)
{
    Depth* depth_buf = framebuffer->api.depth;
    
    #ifdef DMIR_DEPTH_INT32
    Depth invalid_depth = INT32_MIN;
    #else
    Depth invalid_depth = -INFINITY;
    #endif
    Depth max_depth = invalid_depth;
    
    SInt min_x = tx, max_x = tx + STENCIL_SIZE_X - 1;
    SInt min_y = ty, max_y = ty + STENCIL_SIZE_Y - 1;
    for (SInt y = min_y; y <= max_y; y++) {
        SInt i = PIXEL_INDEX(framebuffer, min_x, y);
        for (SInt x = min_x; x <= max_x; x++, i++) {
            if ((depth_buf[i] < DMIR_MAX_DEPTH) & (depth_buf[i] > max_depth)) {
                max_depth = depth_buf[i];
            }
        }
    }
    
    if (max_depth > invalid_depth) {
        tile->depth = max_depth;
    }
}

void write_fragments(RendererInternal* renderer, FramebufferInternal* framebuffer,
    Fragment* fragments, AffineID affine_id)
{
    SInt tile_head = -1;
    SInt tile_tail = -1;
    StencilTile* tiles = framebuffer->stencil_tiles;
    
    for (Fragment* fragment = renderer->fragments; fragment != fragments; fragment++) {
        #if STENCIL_BITS > 0
        {
            #ifdef DMIR_CLEAR_SELF_STENCIL
            // We need to clear self-stencil even if fragments are rejected by depth
            SInt tx = fragment->x >> STENCIL_SHIFT_X;
            SInt ty = fragment->y >> STENCIL_SHIFT_Y;
            SInt ti = tx + (ty << framebuffer->stencil_row_shift);
            StencilTile* tile = tiles + ti;
            if (!tile->update) {
                STENCIL_UPDATE_STENCIL_SET(tile);
                STENCIL_UPDATE_NEXT_SET(tile, -1);
                if (tile_tail < 0) {
                    tile_head = ti;
                } else {
                    STENCIL_UPDATE_NEXT_SET(tiles + tile_tail, ti);
                }
                tile_tail = ti;
            }
            #endif
            
            SInt i = PIXEL_INDEX(framebuffer, fragment->x, fragment->y);
            if (!(fragment->z < framebuffer->api.depth[i])) continue;
            
            #ifdef DMIR_CLEAR_SELF_STENCIL
            // Skip the max depth recalculation if the pixel we're about to
            // overwrite had the default depth or is not at tile's max depth
            if (framebuffer->api.depth[i] == DMIR_MAX_DEPTH) {
                if ((tile->depth == DMIR_MAX_DEPTH) | (fragment->z > tile->depth)) {
                    tile->depth = fragment->z;
                    STENCIL_UPDATE_DEPTH_UNSET(tile);
                }
            } else if (framebuffer->api.depth[i] == tile->depth) {
                STENCIL_UPDATE_DEPTH_SET(tile);
            }
            #endif
            
            framebuffer->api.depth[i] = fragment->z;
        }
        #endif
        
        {
            SInt i = PIXEL_INDEX(framebuffer, fragment->x, fragment->y);
            write_pixel(framebuffer, i, affine_id, fragment->data_ref);
        }
    }
    
    #if (STENCIL_BITS > 0) && defined(DMIR_CLEAR_SELF_STENCIL)
    SInt stencil_row_shift = framebuffer->stencil_row_shift;
    SInt stencil_row_mask = (1 << stencil_row_shift) - 1;
    
    SInt tile_next = tile_head & STENCIL_UPDATE_NEXT;
    while (tile_next < STENCIL_UPDATE_NEXT) {
        SInt ti = tile_next;
        StencilTile* tile = tiles + ti;
        tile_next = STENCIL_UPDATE_NEXT_GET(tile);
        
        if (STENCIL_UPDATE_STENCIL_GET(tile)) {
            tile->scene &= tile->self;
            tile->self = STENCIL_CLEAR;
        }
        
        if (STENCIL_UPDATE_DEPTH_GET(tile)) {
            SInt tx = (ti & stencil_row_mask) << STENCIL_SHIFT_X;
            SInt ty = (ti >> stencil_row_shift) << STENCIL_SHIFT_Y;
            tile_depth_update(framebuffer, tile, tx, ty);
        }
        
        tiles[ti].update = 0;
    }
    #endif
}

void clear_self_stencil(FramebufferInternal* framebuffer, Rect* rect) {
    SInt min_tx = rect->min_x & ~STENCIL_MASK_X;
    SInt max_tx = rect->max_x & ~STENCIL_MASK_X;
    SInt min_ty = rect->min_y & ~STENCIL_MASK_Y;
    SInt max_ty = rect->max_y & ~STENCIL_MASK_Y;
    SInt tile_col = min_tx >> STENCIL_SHIFT_X;
    SInt tile_row = min_ty >> STENCIL_SHIFT_Y;
    for (SInt ty = min_ty; ty <= max_ty; ty += STENCIL_SIZE_Y, tile_row++) {
        StencilTile* tile = framebuffer->stencil_tiles +
            (tile_col + (tile_row << framebuffer->stencil_row_shift));
        for (SInt tx = min_tx; tx <= max_tx; tx += STENCIL_SIZE_X, tile++) {
            tile->self = STENCIL_CLEAR;
        }
    }
}

void render_cage(LocalVariables* v, Subtree* subtree) {
    v->affine_id = subtree->affine_id;
    v->geometry = v->batcher->affine[v->affine_id].geometry;
    
    Effects effects = subtree->effects;
    v->shape = effects.shape;
    
    GridStackItem* stack_start = v->renderer->stack;
    GridStackItem* stack = stack_start;
    
    grid_from_cage(stack->grid, subtree->cage);
    
    float max_scale = 0;
    Bounds bounds = subtree->bounds;
    
    Rect viewport = v->batcher->api.viewport;
    Bounds frustum_bounds = v->batcher->frustum_bounds;
    
    RECT_CLIP(viewport, v->renderer->api.rect);
    frustum_bounds.min_x = viewport.min_x;
    frustum_bounds.max_x = viewport.max_x + 1;
    frustum_bounds.min_y = viewport.min_y;
    frustum_bounds.max_y = viewport.max_y + 1;
    
    SInt viewport_size_x = viewport.max_x - viewport.min_x + 1;
    SInt viewport_size_y = viewport.max_y - viewport.min_y + 1;
    SInt max_splat_size = MAX(viewport_size_x, viewport_size_y) / 2;
    float max_splat_distortion = max_splat_size * 0.25f;
    
    stack->level = 0;
    stack->rect = viewport;
    stack->is_behind = (bounds.min_z <= v->batcher->eye_z);
    stack->is_cube = FALSE;
    
    v->node_ref = subtree->node_ref;
    v->data_ref = subtree->data_ref;
    
    if (v->geometry->evaluate != NULL) {
        stack->traversal.mask[0] = 255;
        stack->traversal.node[0] = v->node_ref;
        stack->traversal.data[0] = v->data_ref;
    } else {
        if (!v->geometry->traverse_start(v->geometry, v->node_ref, v->data_ref, &stack->traversal)) return;
    }
    
    stack->queue = 0;
    
    v->octant = 0;
    v->mask = stack->traversal.mask[v->octant];
    
    v->node_box = subtree->node_box;
    
    v->fragments = v->renderer->fragments;
    
    #ifdef STENCIL_LOCAL_SIZE
    Stencil local_stencil[STENCIL_LOCAL_SIZE];
    #else
    Stencil* local_stencil = NULL;
    #endif
    
    goto skip_initialization;
    
    do {
        if (stack->queue == 0) {
            stack--;
            continue;
        }
        v->octant = stack->queue & 7;
        stack->queue >>= 4;
        v->mask = stack->traversal.mask[v->octant];
        v->node_ref = stack->traversal.node[v->octant];
        node_box_child(&stack->node_box, v->octant, &v->node_box);
        initialize_subgrid(stack->grid, (stack-1)->grid, v->octant);
        calculate_screen_bounds(stack->grid, &bounds, &max_scale);
        
        skip_initialization:;
        
        STAT_INCREMENT(v->framebuffer, DMIR_NODES_CAGE);
        
        if (bounds.min_z >= frustum_bounds.max_z) continue;
        
        SInt intersects_near_plane = (bounds.min_z < frustum_bounds.min_z);
        
        if (intersects_near_plane) {
            if (bounds.max_z <= frustum_bounds.min_z) continue;
            if ((!v->mask) | (stack->level == effects.max_level)) continue;
        }
        
        float dilation = effects.dilation_abs;
        
        SInt intersects_eye_plane = (bounds.min_z <= v->batcher->eye_z);
        
        if (!intersects_near_plane & (effects.dilation_rel > 0)) {
            float bounds_size_x = bounds.max_x - bounds.min_x;
            float bounds_size_y = bounds.max_y - bounds.min_y;
            float bounds_size_max = MAX(bounds_size_x, bounds_size_y);
            float dilation_rel_size = effects.dilation_rel * bounds_size_max * (1 << stack->level);
            dilation += MIN(dilation_rel_size, DILATION_REL_SIZE_MAX);
        }
        
        if (stack->is_behind) enlarge_bounds(&bounds);
        
        // Calculate screen-space bounds (in pixels)
        // (clamps to a sane range if values are too big)
        Rect rect;
        RECT_FROM_BOUNDS(rect, bounds, dilation);
        
        // Calculate node size (in pixels)
        SInt size_x = rect.max_x - rect.min_x;
        SInt size_y = rect.max_y - rect.min_y;
        SInt size_max = MAX(size_x, size_y);
        SInt is_pixel = (size_max == 0);
        SInt is_splat = (!v->mask) | (stack->level == effects.max_level);
        
        // Clamp to viewport
        if (stack->is_behind) {
            RECT_CLIP(rect, viewport);
        } else {
            RECT_CLIP(rect, stack->rect);
        }
        
        // Skip if not visible
        if ((rect.min_x > rect.max_x) | (rect.min_y > rect.max_y)) continue;
        
        if (!intersects_near_plane) {
            Depth depth = z_to_depth(v->batcher, bounds.min_z);
            
            // Do an occlusion check
            if (is_occluded_quad(v->framebuffer, &rect, depth)) continue;
        }
        
        v->is_cube = stack->is_cube;
        
        if ((!v->is_cube) & (v->geometry->evaluate != NULL)) {
            int32_t eval_result = v->geometry->evaluate(v->geometry, &v->node_box, &stack->traversal.data[v->octant]);
            if (eval_result < 0) continue;
            is_splat |= (eval_result == 0);
            v->mask = 255;
        }
        
        if (intersects_near_plane) {
            if (is_pixel | is_splat) continue;
        } else {
            if (v->is_cube) {
                is_splat = FALSE;
                v->mask = 255;
            } else if (is_splat & (effects.shape == DMIR_SHAPE_CUBE)) {
                v->is_cube = TRUE;
                is_splat = FALSE;
                v->mask = 255;
            }
        }
        
        // Calculate the remaining (non-corner) vertices
        // This is also required for calculating matrix axes and starting octant
        calculate_midpoints(stack->grid, v->batcher);
        
        SInt is_subtree = is_pixel | is_splat;
        float projection_distortion = 0;
        // Don't calculate distortion if it's already a pixel or a leaf
        if (!is_subtree & !intersects_eye_plane & (size_max < ORTHO_MAX_SIZE)) {
            projection_distortion = calculate_projection_distortion(stack->grid);
            is_subtree |= (projection_distortion < v->batcher->api.ortho_distortion);
        } else if (is_splat & (size_max > max_splat_size)) {
            // If a splat is big and has significant distortion,
            // it's likely too close and has wildly inaccurate bounds
            // (too large sizes can also cause problems with map mode,
            // which can lead to segfaults or endless cycles)
            projection_distortion = calculate_projection_distortion(stack->grid);
            if (projection_distortion > max_splat_distortion) continue;
            projection_distortion = 0;
        }
        
        if (is_subtree) {
            Effects sub_effects = effects;
            sub_effects.max_level -= stack->level;
            sub_effects.dilation_abs += projection_distortion * 0.5f;
            sub_effects.dilation_rel *= (1 << stack->level);
            
            v->data_ref = stack->traversal.data[v->octant];
            
            OrthoStackItem* ortho_stack = (OrthoStackItem*)(stack+1);
            if (initialize_ortho(v, ortho_stack, stack->grid, &sub_effects)) {
                if (v->geometry->evaluate != NULL) {
                    render_ortho_eval(v, ortho_stack, local_stencil, FALSE, FALSE);
                } else {
                    render_ortho_tree(v, ortho_stack, local_stencil, FALSE, FALSE);
                }
            }
            
            continue;
        }
        
        // Calculate the starting octant
        SInt starting_octant;
        if (v->batcher->is_perspective & (bounds.min_z < v->batcher->eye_z)) {
            starting_octant = calculate_starting_octant_perspective(stack->grid, v->batcher->eye_z);
        } else {
            starting_octant = calculate_starting_octant(stack->grid);
        }
        
        SInt octant_order = calculate_octant_order_grid(stack->grid);
        
        stack++;
        stack->level = (stack-1)->level + 1;
        stack->rect = rect;
        stack->node_box = v->node_box;
        stack->is_behind = intersects_eye_plane;
        stack->is_cube = v->is_cube;
        
        if (v->is_cube | (v->geometry->evaluate != NULL)) {
            v->data_ref = (stack-1)->traversal.data[v->octant];
            for (SInt i = 0; i < 8; i++) {
                stack->traversal.mask[i] = 255;
                stack->traversal.node[i] = v->node_ref;
                stack->traversal.data[i] = v->data_ref;
            }
        } else if (!v->geometry->traverse_next(v->geometry, &(stack-1)->traversal, v->octant, &stack->traversal)) {
            stack--;
            continue;
        }
        
        Queue* queues_forward = lookups.sparse + (((octant_order << 3) | (starting_octant)) << 8);
        stack->queue = queues_forward[v->mask].octants;
    } while (stack > stack_start);
    
    #ifdef DMIR_USE_SPLAT_DEFERRED
    write_fragments(v->renderer, v->framebuffer, v->fragments, v->affine_id);
    #elif (STENCIL_BITS > 0) && defined(DMIR_CLEAR_SELF_STENCIL)
    clear_self_stencil(v->framebuffer, &subtree->rect);
    #endif
}

void dmir_renderer_draw(Renderer* renderer_ptr) {
    RendererInternal* renderer = (RendererInternal*)renderer_ptr;
    FramebufferInternal* framebuffer = (FramebufferInternal*)renderer->api.framebuffer;
    BatcherInternal* batcher = (BatcherInternal*)renderer->api.batcher;
    
    if (!framebuffer || !batcher) return;
    
    SInt size_x = renderer->api.rect.max_x - renderer->api.rect.min_x + 1;
    SInt size_y = renderer->api.rect.max_y - renderer->api.rect.min_y + 1;
    SInt rect_area = size_x * size_y;
    if (rect_area > renderer->fragments_size) {
        renderer->fragments_size = 1 << pow2_ceil(rect_area);
        if (renderer->fragments) free(renderer->fragments);
        renderer->fragments = malloc(renderer->fragments_size * sizeof(Fragment));
    }
    
    LocalVariables v = {
        .batcher = batcher,
        .renderer = renderer,
        .framebuffer = framebuffer,
    };
    
    Subtree* subtrees = batcher->subtrees;
    for (SInt index = batcher->sorted_head; index >= 0; index = subtrees[index].next) {
        render_cage(&v, &batcher->subtrees[index]);
    }
}

static uint64_t visit_recursive(const DMirGeometry* geometry, DMirVisitor* visitor,
    DMirTraversal* src, uint32_t index, DMirNodeBox* node_box, DMirBool reverse)
{
    DMirVisitorNodeInfo info;
    info.is_leaf = (node_box->level >= DMIR_LEVEL_MAX);
    info.is_leaf |= (node_box->level == visitor->level_limit);
    info.is_leaf |= (node_box->level == geometry->max_level);
    info.data_ref = 0;
    info.mask = 0;
    info.subtree_size = 1;
    info.subnode_masks = 0;
    
    DMirTraversal dst;
    uint8_t mask = 255;
    
    if (geometry->evaluate != NULL) {
        int32_t eval_result = geometry->evaluate(geometry, node_box, &info.data_ref);
        if (eval_result < 0) return 0;
        
        info.is_leaf |= (eval_result == 0);
    } else {
        mask = src->mask[index];
        info.data_ref = src->data[index];
        
        info.is_leaf |= (mask == 0);
        
        if (!info.is_leaf) {
            if (!geometry->traverse_next(geometry, src, index, &dst)) return 0;
        }
    }
    
    if (!reverse) {
        visitor->node_count++;
        if (info.is_leaf) visitor->leaf_count++;
        if (node_box->level > visitor->level_max) visitor->level_max = node_box->level;
        
        if (visitor->visit) {
            for (int octant = 0; octant < 8; octant++) {
                info.sub_node_refs[octant] = 0;
                info.sub_data_refs[octant] = 0;
            }
            info.node_box = *node_box;
            if (!visitor->visit(visitor, &info)) return info.subtree_size;
        }
    }
    
    if (!info.is_leaf) {
        int octant_start, octant_end, octant_step;
        if (reverse) {
            octant_start = 7;
            octant_end = -1;
            octant_step = -1;
        } else {
            octant_start = 0;
            octant_end = 8;
            octant_step = 1;
        }
        
        for (int octant = octant_start; octant != octant_end; octant += octant_step) {
            info.sub_node_refs[octant] = 0;
            info.sub_data_refs[octant] = 0;
            
            if ((mask & (1 << octant)) == 0) continue;
            
            DMirNodeBox child_box;
            node_box_child(node_box, octant, &child_box);
            
            uint64_t child_info = visit_recursive(
                geometry, visitor, &dst, octant, &child_box, reverse);
            
            if (child_info) {
                uint64_t child_subtree_size = child_info & (UINT64_MAX >> 8);
                uint8_t child_mask = child_info >> (64-8);
                info.mask |= (1 << octant);
                info.subtree_size += child_subtree_size;
                info.subnode_masks |= (uint64_t)(child_mask) << (octant*8);
                info.sub_node_refs[octant] = visitor->node_ref_new;
                info.sub_data_refs[octant] = visitor->data_ref_new;
            }
        }
        
        if (info.mask == 0) return 0;
    }
    
    if (reverse) {
        visitor->node_count++;
        if (info.is_leaf) visitor->leaf_count++;
        if (node_box->level > visitor->level_max) visitor->level_max = node_box->level;
        
        if (visitor->visit) {
            info.node_box = *node_box;
            visitor->visit(visitor, &info);
        }
    }
    
    return ((uint64_t)(info.mask) << (64-8)) | info.subtree_size;
}

void dmir_visit(const DMirGeometry* geometry, DMirAddress node_ref, DMirVisitor* visitor, DMirBool reverse) {
    DMirTraversal src;
    
    if (geometry->evaluate == NULL) {
        src.node[0] = node_ref;
        if (!geometry->traverse_start(geometry, node_ref, 0, &src)) return;
    }
    
    DMirNodeBox node_box = {.x = 0, .y = 0, .z = 0, .level = 0};
    
    visitor->level_max = 0;
    visitor->node_count = 0;
    visitor->leaf_count = 0;
    visitor->node_ref_new = 0;
    visitor->data_ref_new = 0;
    
    visit_recursive(geometry, visitor, &src, 0, &node_box, reverse);
}
