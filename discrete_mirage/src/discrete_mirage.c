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

#define PTR_OFFSET(array, offset) ((typeof(array))(((char*)(array)) + (offset)))
#define PTR_INDEX(array, index) PTR_OFFSET((array), ((index) * (array##_stride)))

// 32-bit float screen-space coordinates can be halved
// at most 128 times before they become subpixel
#define MAX_STACK_DEPTH 128

// We have to limit the maximum screen-space size
// of relative dilation, because in perspective
// it can become too big near the eye plane
#define DILATION_REL_SIZE_MAX 64

#if DMIR_INT_SIZE == 32
typedef int32_t SInt;
typedef uint32_t UInt;
#elif DMIR_INT_SIZE == 64
typedef int64_t SInt;
typedef uint64_t UInt;
#else
typedef int SInt;
typedef unsigned int UInt;
#endif

#ifdef DMIR_COORD_FIXED
typedef int32_t Coord;
#define SUBPIXEL_BITS DMIR_COORD_FIXED
#define SUBPIXEL_SIZE (1 << SUBPIXEL_BITS)
#define SUBPIXEL_HALF (SUBPIXEL_SIZE >> 1)
#define ORTHO_MAX_SUBDIVISIONS (31 - SUBPIXEL_BITS)
#define ORTHO_LEVEL_LIMIT (28 - SUBPIXEL_BITS)
#define ORTHO_MAX_SIZE (1 << ORTHO_LEVEL_LIMIT)
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

typedef DMirBool Bool;
typedef DMirDepth Depth;
typedef DMirColor Color;
typedef DMirRect Rect;
typedef DMirFrustum Frustum;

typedef DMirOctree Octree;
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

typedef struct GridStackItem {
    ProjectedVertex grid[GRID_SIZE];
    Rect rect;
    uint32_t subnodes[8];
    uint8_t masks[8];
    uint8_t octants[8];
    uint32_t addresses[8];
    SInt count;
    SInt level;
    SInt affine_id;
    SInt is_behind;
} GridStackItem;

typedef struct OrthoStackItem {
    Vector3S deltas[CAGE_SIZE];
    Vector3S center;
    Coord extent_x;
    Coord extent_y;
    Depth extent_z;
    Rect rect;
    uint32_t subnodes[8];
    uint8_t masks[8];
    uint8_t octants[8];
    uint32_t addresses[8];
    SInt count;
    SInt level;
} OrthoStackItem;

typedef struct Fragment {
    int32_t x;
    int32_t y;
    Depth z;
    uint32_t address;
} Fragment;

typedef struct Subtree {
    uint32_t affine_id;
    ProjectedVertex cage[CAGE_SIZE];
    uint32_t address;
    Effects effects;
    Bounds bounds;
    SInt next;
} Subtree;

#ifdef DMIR_ROW_POW2
#define PIXEL_INDEX(buf, x, y) ((x) + ((y) << (buf)->row_shift))
#else
#define PIXEL_INDEX(buf, x, y) ((x) + ((y) * (buf)->size_x))
#endif

UInt pow2_ceil(UInt value) {
    UInt shift = 0;
    while ((1 << shift) < value) shift++;
    return shift;
}

///////////////////
// Lookup tables //
///////////////////

#define XYZ 0
#define XZY 1
#define YXZ 2
#define YZX 3
#define ZXY 4
#define ZYX 5

typedef struct Queue {
    uint32_t octants;
    uint32_t indices;
} Queue;

typedef struct Lookups {
    uint8_t* counts; // mask -> bit count
    uint32_t* octants; // mask, index -> octant
    uint32_t* indices; // mask, octant -> index
    Queue* sparse; // order, mask -> octants & indices
    Queue* packed; // order, mask -> octants & indices
} Lookups;

void lookups_make_octants_indices_counts(Lookups* lookups) {
    lookups->counts = malloc(256 * sizeof(uint8_t));
    lookups->octants = malloc(256 * sizeof(uint32_t));
    lookups->indices = malloc(256 * sizeof(uint32_t));
    
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
        lookups->counts[mask] = count;
        lookups->indices[mask] = o2i;
        lookups->octants[mask] = i2o;
    }
}

void lookups_make_queues(Lookups* lookups) {
    const SInt queues_count = 6*8*256;
    lookups->packed = malloc(queues_count * sizeof(Queue));
    lookups->sparse = malloc(queues_count * sizeof(Queue));
    
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
                uint32_t o2i = lookups->indices[mask];
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
                lookups->packed[(((order << 3) | start_octant) << 8) | mask] = queue;
            }
        }
    }
    
    for (SInt i = 0; i < queues_count; i++) {
        lookups->sparse[i].octants = lookups->sparse[i].indices = lookups->packed[i].octants;
    }
}

Lookups* lookups_make() {
    Lookups* lookups = malloc(sizeof(Lookups));
    
    if (lookups) {
        lookups_make_octants_indices_counts(lookups);
        lookups_make_queues(lookups);
    }
    
    return lookups;
}

void lookups_free(Lookups* lookups) {
    if (lookups->counts) free(lookups->counts);
    if (lookups->octants) free(lookups->octants);
    if (lookups->indices) free(lookups->indices);
    if (lookups->sparse) free(lookups->sparse);
    if (lookups->packed) free(lookups->packed);
    
    free(lookups);
}

/////////////////////////////////////
// Rendering-related functionality //
/////////////////////////////////////

typedef struct BatcherInternal {
    Batcher api;
    Lookups* lookups;
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

static inline SInt is_occluded_quad(Framebuffer* framebuffer, Rect* rect, Depth depth) {
    #ifndef DMIR_USE_OCCLUSION
    return FALSE;
    #endif
    for (SInt y = rect->min_y; y <= rect->max_y; y++) {
        SInt row = PIXEL_INDEX(framebuffer, 0, y);
        for (SInt x = rect->min_x; x <= rect->max_x; x++) {
            if (framebuffer->depth[row+x] > depth) return FALSE;
        }
        #ifdef DMIR_SKIP_OCCLUDED_ROWS
        rect->min_y = y + 1;
        #endif
    }
    return TRUE;
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
    (rect).min_x = (int32_t)((bounds).min_x - (dilation));\
    (rect).max_x = (int32_t)((bounds).max_x + (dilation));\
    (rect).min_y = (int32_t)((bounds).min_y - (dilation));\
    (rect).max_y = (int32_t)((bounds).max_y + (dilation));\
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

SInt calculate_starting_octant_ortho(Vector3S* matrix) {
    SInt bit_x = (matrix[1].y * matrix[2].x <= matrix[1].x * matrix[2].y ? 0 : 1);
    SInt bit_y = (matrix[2].y * matrix[0].x <= matrix[2].x * matrix[0].y ? 0 : 2);
    SInt bit_z = (matrix[0].y * matrix[1].x <= matrix[0].x * matrix[1].y ? 0 : 4);
    return bit_x | bit_y | bit_z;
}

SInt calculate_octant_order(Vector3S* matrix) {
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

static inline void write_pixel(Framebuffer* framebuffer, SInt i,
    int32_t affine_id, uint32_t address, Octree* octree)
{
    #ifdef DMIR_USE_SPLAT_COLOR
    uint8_t* data_ptr = PTR_INDEX(octree->data, address);
    Color* color_ptr = (Color*)data_ptr;
    framebuffer->color[i] = *color_ptr;
    #else
    framebuffer->voxel[i].affine_id = affine_id;
    framebuffer->voxel[i].address = address;
    #endif
}

static inline void splat_pixel(Framebuffer* framebuffer, SInt x, SInt y,
    Depth depth, int32_t affine_id, uint32_t address, Octree* octree)
{
    SInt i = PIXEL_INDEX(framebuffer, x, y);
    
    if (depth < framebuffer->depth[i]) {
        framebuffer->depth[i] = depth;
        write_pixel(framebuffer, i, affine_id, address, octree);
    }
}

static inline void splat_deferred(Framebuffer* framebuffer, Fragment** fragments,
    int32_t x, int32_t y, Depth depth, uint32_t address)
{
    SInt i = PIXEL_INDEX(framebuffer, x, y);
    
    if (depth < framebuffer->depth[i]) {
        framebuffer->depth[i] = depth;
        
        fragments[0]->x = x;
        fragments[0]->y = y;
        fragments[0]->z = depth;
        fragments[0]->address = address;
        fragments[0]++;
    }
}

SInt calculate_max_level(Vector3F* fmatrix) {
    float gap_x = fabsf(fmatrix[0].x) + fabsf(fmatrix[1].x) + fabsf(fmatrix[2].x);
    float gap_y = fabsf(fmatrix[0].y) + fabsf(fmatrix[1].y) + fabsf(fmatrix[2].y);
    float gap_max = 2 * MAX(gap_x, gap_y);
    for (SInt max_level = 0; max_level <= ORTHO_LEVEL_LIMIT; max_level++) {
        if (gap_max < (1 << max_level)) return max_level;
    }
    return -1; // too big; can't render
}

SInt calculate_ortho_matrix(BatcherInternal* batcher, ProjectedVertex* grid, Vector3S* matrix) {
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
    
    matrix[0].z = z_to_depth(batcher, fmatrix[0].z);
    matrix[1].z = dz_to_depth(batcher, fmatrix[1].z);
    matrix[2].z = dz_to_depth(batcher, fmatrix[2].z);
    matrix[3].z = dz_to_depth(batcher, fmatrix[3].z);
    
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

void calculate_ortho_extent(Vector3S* matrix, Vector3S* extent) {
    #ifdef DMIR_COORD_FIXED
    extent->x = (abs(matrix[0].x) + abs(matrix[1].x) + abs(matrix[2].x)) * 2;
    extent->y = (abs(matrix[0].y) + abs(matrix[1].y) + abs(matrix[2].y)) * 2;
    #else
    extent->x = (fabsf(matrix[0].x) + fabsf(matrix[1].x) + fabsf(matrix[2].x)) * 2;
    extent->y = (fabsf(matrix[0].y) + fabsf(matrix[1].y) + fabsf(matrix[2].y)) * 2;
    #endif
    
    #ifdef DMIR_DEPTH_INT32
    extent->z = (abs(matrix[0].z) + abs(matrix[1].z) + abs(matrix[2].z)) * 2;
    #else
    extent->z = (fabsf(matrix[0].z) + fabsf(matrix[1].z) + fabsf(matrix[2].z)) * 2;
    #endif
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

void render_ortho(RendererInternal* renderer, BatcherInternal* batcher, Framebuffer* framebuffer,
    uint32_t affine_id, Octree* octree, uint32_t address, Effects effects,
    ProjectedVertex* grid, OrthoStackItem* stack_start)
{
    Vector3S matrix[4];
    SInt max_level = calculate_ortho_matrix(batcher, grid, matrix);
    if (max_level < 0) return;
    
    effects.max_level = (effects.max_level < 0 ? max_level : MIN(effects.max_level, max_level));
    
    Vector3S extent;
    calculate_ortho_extent(matrix, &extent);
    
    Coord dilation = (Coord)(effects.dilation_abs * SUBPIXEL_SIZE);
    dilation += (Coord)(effects.dilation_rel * 2 * MAX(extent.x, extent.y));
    
    OrthoStackItem* stack = stack_start;
    
    calculate_ortho_deltas(stack->deltas, matrix, 2);
    
    stack->extent_x = extent.x;
    stack->extent_y = extent.y;
    stack->extent_z = extent.z;
    
    for (SInt level = 1; level <= effects.max_level; level++) {
        stack[level].extent_x = COORD_HALVE(stack[level-1].extent_x);
        stack[level].extent_y = COORD_HALVE(stack[level-1].extent_y);
        stack[level].extent_z = DEPTH_HALVE(stack[level-1].extent_z);
        for (SInt octant = 0; octant < 8; octant++) {
            stack[level].deltas[octant].x = COORD_HALVE(stack[level-1].deltas[octant].x);
            stack[level].deltas[octant].y = COORD_HALVE(stack[level-1].deltas[octant].y);
            stack[level].deltas[octant].z = DEPTH_HALVE(stack[level-1].deltas[octant].z);
        }
    }
    
    for (SInt level = 0; level <= effects.max_level; level++) {
        stack[level].level = level;
        stack[level].extent_x += dilation;
        stack[level].extent_y += dilation;
    }
    
    stack->rect = renderer->api.rect;
    
    Vector3S position;
    position.x = matrix[3].x;
    position.y = matrix[3].y;
    position.z = matrix[3].z;
    
    // Calculate the starting octant
    SInt octant_order = calculate_octant_order(matrix);
    SInt starting_octant = calculate_starting_octant_ortho(matrix);
    SInt starting_octant_reverse = (~starting_octant) & 7;
    
    Queue* queues = (octree->is_packed ? batcher->lookups->packed : batcher->lookups->sparse);
    Queue* queues_forward = queues + (((octant_order << 3) | (starting_octant ^ 0b000)) << 8);
    Queue* queues_reverse = queues + (((octant_order << 3) | (starting_octant ^ 0b111)) << 8);
    
    SInt mask = *PTR_INDEX(octree->mask, address);
    uint32_t child_start = *PTR_INDEX(octree->addr, address);
    
    Depth min_depth = z_to_depth(batcher, batcher->frustum_bounds.min_z);
    Depth max_depth = z_to_depth(batcher, batcher->frustum_bounds.max_z);
    
    Fragment* fragments = renderer->fragments;
    
    goto grid_initialized;
    
    do {
        {
            // Pop an entry from the stack
            if (stack->count == 0) {
                stack--;
                continue;
            }
            stack->count--;
            child_start = stack->subnodes[stack->count];
            mask = stack->masks[stack->count];
            address = stack->addresses[stack->count];
            
            SInt octant = stack->octants[stack->count];
            position.x = stack->center.x + stack->deltas[octant].x;
            position.y = stack->center.y + stack->deltas[octant].y;
            position.z = stack->center.z + stack->deltas[octant].z;
        }
        
        grid_initialized:;
        
        Depth depth = position.z - stack->extent_z;
        
        if (depth >= max_depth) continue;
        
        SInt intersects_near_plane = (depth < min_depth);
        
        if (intersects_near_plane) {
            if ((!mask) | (stack->level == effects.max_level)) continue;
        }
        
        // Calculate screen-space bounds (in pixels)
        Rect rect;
        rect.min_x = COORD_TO_PIXEL(position.x - stack->extent_x);
        rect.max_x = COORD_TO_PIXEL(position.x + stack->extent_x);
        rect.min_y = COORD_TO_PIXEL(position.y - stack->extent_y);
        rect.max_y = COORD_TO_PIXEL(position.y + stack->extent_y);
        
        // Calculate node size (in pixels)
        SInt size_x = rect.max_x - rect.min_x;
        SInt size_y = rect.max_y - rect.min_y;
        SInt size_max = MAX(size_x, size_y);
        
        // Clamp to viewport
        RECT_CLIP(rect, stack->rect);
        
        // Skip if not visible
        if ((rect.min_x > rect.max_x) | (rect.min_y > rect.max_y)) continue;
        
        SInt is_pixel = (size_max == 0);
        SInt is_splat = (!mask) | (stack->level == effects.max_level);
        
        if (intersects_near_plane) {
            if (is_pixel | is_splat) continue;
        } else {
            #ifdef DMIR_USE_SPLAT_PIXEL
            // Splat if size is 1 pixel
            if (is_pixel) {
                #ifdef DMIR_USE_SPLAT_DEFERRED
                splat_deferred(framebuffer, &fragments, rect.min_x, rect.min_y, position.z, address);
                #else
                splat_pixel(framebuffer, rect.min_x, rect.min_y, position.z, affine_id, address, octree);
                #endif
                continue;
            }
            #else
            is_splat |= is_pixel;
            #endif
            
            // Splat if this is a leaf node or reached max displayed level
            if (is_splat) {
                for (SInt y = rect.min_y; y <= rect.max_y; y++) {
                    for (SInt x = rect.min_x; x <= rect.max_x; x++) {
                        #ifdef DMIR_USE_SPLAT_DEFERRED
                        splat_deferred(framebuffer, &fragments, x, y, position.z, address);
                        #else
                        splat_pixel(framebuffer, x, y, position.z, affine_id, address, octree);
                        #endif
                    }
                }
                continue;
            }
            
            // Do an occlusion check
            if (is_occluded_quad(framebuffer, &rect, depth)) continue;
            
            #ifdef DMIR_USE_BLIT_AT_2X2
            if (size_max <= 1) {
                Vector3S* deltas = (stack+1)->deltas;
                Queue queue = queues_reverse[mask];
                for (; queue.indices != 0; queue.indices >>= 4, queue.octants >>= 4) {
                    SInt octant = (queue.octants & 7);
                    address = child_start + (queue.indices & 7);
                    
                    SInt x = COORD_TO_PIXEL(position.x + deltas[octant].x);
                    if ((x < stack->rect.min_x) | (x > stack->rect.max_x)) continue;
                    
                    SInt y = COORD_TO_PIXEL(position.y + deltas[octant].y);
                    if ((y < stack->rect.min_y) | (y > stack->rect.max_y)) continue;
                    
                    Depth z = position.z + deltas[octant].z;
                    
                    #ifdef DMIR_USE_SPLAT_DEFERRED
                    splat_deferred(framebuffer, &fragments, x, y, z, address);
                    #else
                    splat_pixel(framebuffer, x, y, z, affine_id, address, octree);
                    #endif
                }
                continue;
            }
            #endif
        }
        
        // Push non-empty children on the stack
        stack++;
        stack->count = 0;
        stack->rect = rect;
        stack->center = position;
        
        Queue queue = queues_reverse[mask];
        for (; queue.indices != 0; queue.indices >>= 4, queue.octants >>= 4) {
            address = child_start + (queue.indices & 7);
            stack->octants[stack->count] = (queue.octants & 7);
            stack->addresses[stack->count] = address;
            stack->subnodes[stack->count] = *PTR_INDEX(octree->addr, address);
            stack->masks[stack->count] = *PTR_INDEX(octree->mask, address);
            stack->count++;
        }
    } while (stack > stack_start);
    
    #ifdef DMIR_USE_SPLAT_DEFERRED
    for (Fragment* fragment = renderer->fragments; fragment != fragments; fragment++) {
        SInt i = PIXEL_INDEX(framebuffer, fragment->x, fragment->y);
        write_pixel(framebuffer, i, affine_id, fragment->address, octree);
    }
    #endif
}

///////////////////////////////////////////
// Public API and some related functions //
///////////////////////////////////////////

Framebuffer* dmir_framebuffer_make(uint32_t size_x, uint32_t size_y) {
    Framebuffer* framebuffer = malloc(sizeof(Framebuffer));
    
    if (framebuffer) {
        framebuffer->depth = NULL;
        framebuffer->voxel = NULL;
        framebuffer->color = NULL;
        
        dmir_framebuffer_resize(framebuffer, size_x, size_y);
    }
    
    return framebuffer;
}

void framebuffer_free_channels(Framebuffer* framebuffer) {
    if (framebuffer->depth) free(framebuffer->depth);
    if (framebuffer->voxel) free(framebuffer->voxel);
    if (framebuffer->color) free(framebuffer->color);
}

void dmir_framebuffer_free(Framebuffer* framebuffer) {
    framebuffer_free_channels(framebuffer);
    
    free(framebuffer);
}

void dmir_framebuffer_resize(Framebuffer* framebuffer, uint32_t size_x, uint32_t size_y) {
    framebuffer_free_channels(framebuffer);
    
    framebuffer->size_x = size_x;
    framebuffer->size_y = size_y;
    framebuffer->row_shift = pow2_ceil(size_x);
    
    SInt buffer_size = dmir_row_size(framebuffer) * framebuffer->size_y;
    buffer_size = MAX(buffer_size, 1); // safeguard if width or height is 0
    framebuffer->depth = malloc(buffer_size * sizeof(Depth));
    framebuffer->voxel = malloc(buffer_size * sizeof(VoxelRef));
    framebuffer->color = malloc(buffer_size * sizeof(Color));
    
    dmir_framebuffer_clear(framebuffer);
}

void dmir_framebuffer_clear(Framebuffer* framebuffer) {
    Color color = {.r = 0, .g = 0, .b = 0};
    VoxelRef voxel = {.affine_id = -1, .address = 0};
    
    for (SInt y = 0; y < framebuffer->size_y; y++) {
        Depth* depth_row = framebuffer->depth + PIXEL_INDEX(framebuffer, 0, y);
        VoxelRef* voxel_row = framebuffer->voxel + PIXEL_INDEX(framebuffer, 0, y);
        Color* color_row = framebuffer->color + PIXEL_INDEX(framebuffer, 0, y);
        for (SInt x = 0; x < framebuffer->size_x; x++) {
            depth_row[x] = DMIR_MAX_DEPTH;
            voxel_row[x] = voxel;
            color_row[x] = color;
        }
    }
}

Bool dmir_is_occluded_quad(Framebuffer* framebuffer, Rect rect, Depth depth) {
    return (Bool)is_occluded_quad(framebuffer, &rect, depth);
}

Batcher* dmir_batcher_make() {
    BatcherInternal* batcher = malloc(sizeof(BatcherInternal));
    
    if (batcher) {
        batcher->lookups = lookups_make();
        
        batcher->affine_size = 1024;
        batcher->affine = malloc(batcher->affine_size * sizeof(AffineInfo));
        
        batcher->subtrees_size = 2048;
        batcher->subtrees = malloc(batcher->subtrees_size * sizeof(Subtree));
        
        batcher->stack = malloc(MAX_STACK_DEPTH * sizeof(GridStackItem));
        
        batcher->api.split_factor = 0.125f;
        batcher->api.affine_distortion = 1;
        batcher->api.ortho_distortion = 1;
        
        dmir_batcher_reset((Batcher*)batcher, DMIR_RECT_EMPTY, DMIR_FRUSTUM_DEFAULT);
    }
    
    return (Batcher*)batcher;
}

void dmir_batcher_free(Batcher* batcher_ptr) {
    BatcherInternal* batcher = (BatcherInternal*)batcher_ptr;
    
    if (batcher->lookups) lookups_free(batcher->lookups);
    
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
        
        batcher->offset_x = viewport.min_x + half_x;
        batcher->offset_y = viewport.min_y + half_y;
        
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
    uint32_t group, Octree* octree, ProjectedVertex* grid)
{
    if (batcher->affine_count == batcher->affine_size) {
        batcher->affine_size *= 2;
        batcher->affine = realloc(batcher->affine, batcher->affine_size * sizeof(AffineInfo));
    }
    SInt affine_id = batcher->affine_count;
    AffineInfo* affine_info = batcher->affine + affine_id;
    affine_info->group = group;
    affine_info->octree = octree;
    Vector3F* matrix = (Vector3F*)(&affine_info->matrix);
    matrix[0] = normalized_matrix_axis(grid, 1, 0, 0);
    matrix[1] = normalized_matrix_axis(grid, 0, 1, 0);
    matrix[2] = normalized_matrix_axis(grid, 0, 0, 1);
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

void dmir_batcher_add(Batcher* batcher_ptr, Framebuffer* framebuffer,
    uint32_t group, float* cage_ptr, Octree* octree, uint32_t root, Effects effects)
{
    BatcherInternal* batcher = (BatcherInternal*)batcher_ptr;
    
    GridStackItem* stack_start = batcher->stack;
    GridStackItem* stack = stack_start;
    
    initialize_grid(stack->grid, (Vector3F*)cage_ptr, batcher);
    
    float max_scale;
    Bounds bounds;
    calculate_screen_bounds(stack->grid, &bounds, &max_scale);
    
    if (!isfinite(bounds.max_z - bounds.min_z)) return;
    
    if (octree->max_level >= 0) {
        if (effects.max_level < 0) {
            effects.max_level = octree->max_level;
        } else {
            MIN_UPDATE(effects.max_level, octree->max_level);
        }
    }
    
    effects.dilation_abs = MAX(effects.dilation_abs, 0) - 0.5f;
    effects.dilation_rel = CLAMP(effects.dilation_rel, 0, 1) * 0.5f;
    
    Rect viewport = batcher->api.viewport;
    Bounds frustum_bounds = batcher->frustum_bounds;
    
    SInt viewport_size_x = viewport.max_x - viewport.min_x + 1;
    SInt viewport_size_y = viewport.max_y - viewport.min_y + 1;
    SInt viewport_size_max = MAX(viewport_size_x, viewport_size_y);
    float split_size = batcher->api.split_factor * viewport_size_max;
    SInt max_subtree_size = (SInt)CLAMP(split_size, 0, INT32_MAX);
    MAX_UPDATE(max_subtree_size, (SInt)(effects.dilation_abs * 2 + 1));
    
    uint32_t address = root;
    SInt mask = *PTR_INDEX(octree->mask, address);
    uint32_t child_start = *PTR_INDEX(octree->addr, address);
    
    stack->level = 0;
    stack->affine_id = -1;
    stack->rect = viewport;
    stack->is_behind = (bounds.min_z <= batcher->eye_z);
    
    Queue* queues = (octree->is_packed ? batcher->lookups->packed : batcher->lookups->sparse);
    
    goto grid_initialized;
    
    do {
        {
            // Pop an entry from the stack
            if (stack->count == 0) {
                stack--;
                continue;
            }
            stack->count--;
            child_start = stack->subnodes[stack->count];
            mask = stack->masks[stack->count];
            address = stack->addresses[stack->count];
            
            // Copy corner vertices from the parent grid's sub-octant vertices
            SInt octant = stack->octants[stack->count];
            initialize_subgrid(stack->grid, (stack-1)->grid, octant);
        }
        
        // Find the min/max of the corner vertices
        calculate_screen_bounds(stack->grid, &bounds, &max_scale);
        
        grid_initialized:;
        
        if (bounds.min_z >= frustum_bounds.max_z) continue;
        
        SInt intersects_near_plane = (bounds.min_z < frustum_bounds.min_z);
        
        if (intersects_near_plane) {
            if (bounds.max_z <= frustum_bounds.min_z) continue;
            if ((!mask) | (stack->level == effects.max_level)) continue;
        }
        
        float dilation = effects.dilation_abs;
        
        SInt intersects_eye_plane = (bounds.min_z <= batcher->eye_z);
        
        if (intersects_eye_plane) {
            RECT_CLIP(bounds, frustum_bounds);
        } else if (!intersects_near_plane) {
            float bounds_size_x = bounds.max_x - bounds.min_x;
            float bounds_size_y = bounds.max_y - bounds.min_y;
            float bounds_size_max = MAX(bounds_size_x, bounds_size_y);
            float dilation_rel_size = effects.dilation_rel * bounds_size_max * (1 << stack->level);
            dilation += MIN(dilation_rel_size, DILATION_REL_SIZE_MAX);
        }
        
        // Calculate screen-space bounds (in pixels)
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
                stack->affine_id = batcher_add_affine(batcher, group, octree, stack->grid);
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
                cage_from_grid(stack->grid, subtree->cage);
                subtree->address = address;
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
        
        Queue* queues_reverse = queues + (((octant_order << 3) | (starting_octant ^ 0b111)) << 8);
        
        // Push non-empty children on the stack
        stack++;
        stack->count = 0;
        stack->level = (stack-1)->level + 1;
        stack->rect = rect;
        stack->is_behind = intersects_eye_plane;
        stack->affine_id = (stack-1)->affine_id;
        
        Queue queue = queues_reverse[mask];
        for (; queue.indices != 0; queue.indices >>= 4, queue.octants >>= 4) {
            address = child_start + (queue.indices & 7);
            stack->octants[stack->count] = (queue.octants & 7);
            stack->addresses[stack->count] = address;
            stack->subnodes[stack->count] = *PTR_INDEX(octree->addr, address);
            stack->masks[stack->count] = *PTR_INDEX(octree->mask, address);
            stack->count++;
        }
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

void render_cage(RendererInternal* renderer, BatcherInternal* batcher, Framebuffer* framebuffer,
    Subtree* subtree)
{
    uint32_t affine_id = subtree->affine_id;
    Octree* octree = batcher->affine[affine_id].octree;
    Effects effects = subtree->effects;
    
    GridStackItem* stack_start = renderer->stack;
    GridStackItem* stack = stack_start;
    
    grid_from_cage(stack->grid, subtree->cage);
    
    float max_scale = 0;
    Bounds bounds = subtree->bounds;
    
    Rect viewport = batcher->api.viewport;
    Bounds frustum_bounds = batcher->frustum_bounds;
    
    RECT_CLIP(viewport, renderer->api.rect);
    frustum_bounds.min_x = viewport.min_x;
    frustum_bounds.max_x = viewport.max_x + 1;
    frustum_bounds.min_y = viewport.min_y;
    frustum_bounds.max_y = viewport.max_y + 1;
    
    uint32_t address = subtree->address;
    SInt mask = *PTR_INDEX(octree->mask, address);
    uint32_t child_start = *PTR_INDEX(octree->addr, address);
    
    stack->level = 0;
    stack->rect = viewport;
    stack->is_behind = (bounds.min_z <= batcher->eye_z);
    
    Queue* queues = (octree->is_packed ? batcher->lookups->packed : batcher->lookups->sparse);
    
    goto grid_initialized;
    
    do {
        {
            // Pop an entry from the stack
            if (stack->count == 0) {
                stack--;
                continue;
            }
            stack->count--;
            child_start = stack->subnodes[stack->count];
            mask = stack->masks[stack->count];
            address = stack->addresses[stack->count];
            
            // Copy corner vertices from the parent grid's sub-octant vertices
            SInt octant = stack->octants[stack->count];
            initialize_subgrid(stack->grid, (stack-1)->grid, octant);
        }
        
        // Find the min/max of the corner vertices
        calculate_screen_bounds(stack->grid, &bounds, &max_scale);
        
        grid_initialized:;
        
        if (bounds.min_z >= frustum_bounds.max_z) continue;
        
        SInt intersects_near_plane = (bounds.min_z < frustum_bounds.min_z);
        
        if (intersects_near_plane) {
            if (bounds.max_z <= frustum_bounds.min_z) continue;
            if ((!mask) | (stack->level == effects.max_level)) continue;
        }
        
        float dilation = effects.dilation_abs;
        
        SInt intersects_eye_plane = (bounds.min_z <= batcher->eye_z);
        
        if (intersects_eye_plane) {
            RECT_CLIP(bounds, frustum_bounds);
        } else if (!intersects_near_plane) {
            float bounds_size_x = bounds.max_x - bounds.min_x;
            float bounds_size_y = bounds.max_y - bounds.min_y;
            float bounds_size_max = MAX(bounds_size_x, bounds_size_y);
            float dilation_rel_size = effects.dilation_rel * bounds_size_max * (1 << stack->level);
            dilation += MIN(dilation_rel_size, DILATION_REL_SIZE_MAX);
        }
        
        // Calculate screen-space bounds (in pixels)
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
        
        if (intersects_near_plane) {
            if (is_pixel | is_splat) continue;
        } else {
            Depth depth = z_to_depth(batcher, bounds.min_z);
            
            #ifndef DMIR_USE_ORTHO
            #ifdef DMIR_USE_SPLAT_PIXEL
            // Splat if size is 1 pixel
            if (is_pixel) {
                splat_pixel(framebuffer, rect.min_x, rect.min_y, depth, affine_id, address, octree);
                continue;
            }
            #else
            is_splat |= is_pixel;
            #endif
            
            // Splat if this is a leaf node or reached max displayed level
            if (is_splat) {
                for (SInt y = rect.min_y; y <= rect.max_y; y++) {
                    for (SInt x = rect.min_x; x <= rect.max_x; x++) {
                        splat_pixel(framebuffer, x, y, depth, affine_id, address, octree);
                    }
                }
                continue;
            }
            #endif
            
            // Do an occlusion check
            if (is_occluded_quad(framebuffer, &rect, depth)) continue;
        }
        
        // Calculate the remaining (non-corner) vertices
        // This is also required for calculating matrix axes and starting octant
        calculate_midpoints(stack->grid, batcher);
        
        #ifdef DMIR_USE_ORTHO
        SInt is_subtree = is_pixel | is_splat;
        float projection_distortion = 0;
        if ((!intersects_eye_plane) & (size_max < ORTHO_MAX_SIZE)) {
            projection_distortion = calculate_projection_distortion(stack->grid);
            is_subtree |= (projection_distortion < batcher->api.ortho_distortion);
        }
        
        if (is_subtree) {
            Effects sub_effects = effects;
            sub_effects.max_level -= stack->level;
            sub_effects.dilation_abs += projection_distortion * 0.5f;
            sub_effects.dilation_rel *= (1 << stack->level);
            
            render_ortho(renderer, batcher, framebuffer, affine_id, octree, address, sub_effects,
                stack->grid, (OrthoStackItem*)(stack+1));
            continue;
        }
        #endif
        
        // Calculate the starting octant
        SInt starting_octant;
        if (batcher->is_perspective & (bounds.min_z < batcher->eye_z)) {
            starting_octant = calculate_starting_octant_perspective(stack->grid, batcher->eye_z);
        } else {
            starting_octant = calculate_starting_octant(stack->grid);
        }
        
        SInt octant_order = calculate_octant_order_grid(stack->grid);
        
        Queue* queues_reverse = queues + (((octant_order << 3) | (starting_octant ^ 0b111)) << 8);
        
        // Push non-empty children on the stack
        stack++;
        stack->count = 0;
        stack->level = (stack-1)->level + 1;
        stack->rect = rect;
        stack->is_behind = intersects_eye_plane;
        
        Queue queue = queues_reverse[mask];
        for (; queue.indices != 0; queue.indices >>= 4, queue.octants >>= 4) {
            address = child_start + (queue.indices & 7);
            stack->octants[stack->count] = (queue.octants & 7);
            stack->addresses[stack->count] = address;
            stack->subnodes[stack->count] = *PTR_INDEX(octree->addr, address);
            stack->masks[stack->count] = *PTR_INDEX(octree->mask, address);
            stack->count++;
        }
    } while (stack > stack_start);
}

void dmir_renderer_draw(Renderer* renderer_ptr) {
    RendererInternal* renderer = (RendererInternal*)renderer_ptr;
    Framebuffer* framebuffer = renderer->api.framebuffer;
    BatcherInternal* batcher = (BatcherInternal*)renderer->api.batcher;
    
    SInt size_x = renderer->api.rect.max_x - renderer->api.rect.min_x + 1;
    SInt size_y = renderer->api.rect.max_y - renderer->api.rect.min_y + 1;
    SInt rect_area = size_x * size_y;
    if (rect_area > renderer->fragments_size) {
        renderer->fragments_size = 1 << pow2_ceil(rect_area);
        renderer->fragments = realloc(renderer->fragments, renderer->fragments_size * sizeof(Fragment));
    }
    
    Subtree* subtrees = batcher->subtrees;
    
    for (SInt index = batcher->sorted_head; index >= 0; index = subtrees[index].next) {
        render_cage(renderer, batcher, framebuffer, &batcher->subtrees[index]);
    }
}
