// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

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

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) < (b) ? (b) : (a))
#define CLAMP(value, low, high) ((value) > (low) ? ((value) < (high) ? (value) : (high)) : (low))

#ifdef DMIR_MINMAX_IF
#define MIN_UPDATE(var, value) if ((value) < var) var = (value);
#define MAX_UPDATE(var, value) if ((value) > var) var = (value);
#else
#define MIN_UPDATE(var, value) var = ((value) < var ? (value) : var);
#define MAX_UPDATE(var, value) var = ((value) > var ? (value) : var);
#endif

#define PTR_OFFSET(array, offset) ((typeof(array))(((char*)(array)) + (offset)))
#define PTR_INDEX(array, index) PTR_OFFSET((array), ((index) * (array##_stride)))

// 32-bit float screen-space coordinates can be halved
// at most 128 times before they become subpixel
#define MAX_STACK_DEPTH 128

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
    uint32_t node;
    SInt count;
    SInt level;
    SInt affine_id;
    SInt is_behind;
} GridStackItem;

typedef struct Subtree {
    uint32_t affine_id;
    ProjectedVertex cage[CAGE_SIZE];
    uint32_t address;
    Effects effects;
    Bounds bounds;
} Subtree;

typedef struct BatcherInternal {
    Batcher api;
    AffineInfo* affine;
    Subtree* subtrees;
    Subtree** sorted;
    GridStackItem* stack;
    SInt affine_size;
    SInt subtrees_size;
    SInt affine_count;
    SInt subtrees_count;
    SInt renderable_count;
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
} RendererInternal;

#ifdef DMIR_DEPTH_INT32
static inline Depth z_to_depth(BatcherInternal* batcher, float z) {
    return (Depth)((z - batcher->api.frustum.min_depth) * batcher->depth_factor);
}
#else
static inline Depth z_to_depth(BatcherInternal* batcher, float z) {
    return (Depth)z;
}
#endif

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

static inline SInt is_occluded_quad(Framebuffer* framebuffer,
    Rect rect, Depth depth)
{
    for (SInt y = rect.min_y; y <= rect.max_y; y++) {
        SInt row = PIXEL_INDEX(framebuffer, 0, y);
        for (SInt x = rect.min_x; x <= rect.max_x; x++) {
            if (framebuffer->depth[row+x] > depth) return FALSE;
        }
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
#define RECT_FROM_BOUNDS_F(rect, bounds) {\
    (rect).min_x = (int32_t)((bounds).min_x+0.5f);\
    (rect).max_x = (int32_t)((bounds).max_x-0.5f);\
    (rect).min_y = (int32_t)((bounds).min_y+0.5f);\
    (rect).max_y = (int32_t)((bounds).max_y-0.5f);\
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
#define GRID_AXIS_CAGE(grid, x, y, z, component) 0.5f * (\
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

static inline void splat_pixel(Framebuffer* framebuffer, SInt x, SInt y,
    Depth depth, int32_t affine_id, uint32_t address, Octree* octree)
{
    SInt i = PIXEL_INDEX(framebuffer, x, y);
    
    if (depth < framebuffer->depth[i]) {
        framebuffer->depth[i] = depth;
        
        #ifdef DMIR_USE_SPLAT_COLOR
        uint8_t* data_ptr = PTR_INDEX(octree->data, address);
        Color* color_ptr = (Color*)data_ptr;
        framebuffer->color[i] = *color_ptr;
        #else
        framebuffer->voxel[i].affine_id = affine_id;
        framebuffer->voxel[i].address = address;
        #endif
    }
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
    return (Bool)is_occluded_quad(framebuffer, rect, depth);
}

Batcher* dmir_batcher_make() {
    BatcherInternal* batcher = malloc(sizeof(BatcherInternal));
    
    if (batcher) {
        batcher->affine_size = 1024;
        batcher->affine = malloc(batcher->affine_size * sizeof(AffineInfo));
        
        batcher->subtrees_size = 2048;
        batcher->subtrees = malloc(batcher->subtrees_size * sizeof(Subtree));
        batcher->sorted = malloc(batcher->subtrees_size * sizeof(Subtree*));
        
        batcher->stack = malloc(MAX_STACK_DEPTH * sizeof(GridStackItem));
        
        batcher->api.split_factor = 0.125f;
        batcher->api.affine_distortion = 1;
        
        dmir_batcher_reset((Batcher*)batcher, DMIR_RECT_EMPTY, DMIR_FRUSTUM_DEFAULT);
    }
    
    return (Batcher*)batcher;
}

void dmir_batcher_free(Batcher* batcher_ptr) {
    BatcherInternal* batcher = (BatcherInternal*)batcher_ptr;
    
    if (batcher->affine) free(batcher->affine);
    
    if (batcher->subtrees) free(batcher->subtrees);
    if (batcher->sorted) free(batcher->sorted);
    
    if (batcher->stack) free(batcher->stack);
    
    free(batcher);
}

void dmir_batcher_reset(Batcher* batcher_ptr, Rect viewport, Frustum frustum) {
    BatcherInternal* batcher = (BatcherInternal*)batcher_ptr;
    
    batcher->affine_count = 0;
    batcher->subtrees_count = 0;
    batcher->renderable_count = 0;
    
    batcher->api.viewport = viewport;
    batcher->api.frustum = frustum;
    
    batcher->api.rect = DMIR_RECT_EMPTY;
    
    float half_x = (viewport.max_x - viewport.min_x + 1) * 0.5f;
    float half_y = (viewport.max_y - viewport.min_y + 1) * 0.5f;
    
    batcher->scale_xy = ((half_x > half_y) ? half_x : half_y) / frustum.focal_extent;
    batcher->scale_c = 1 - frustum.perspective;
    batcher->scale_z = frustum.perspective / frustum.focal_depth;
    
    batcher->offset_x = viewport.min_x + half_x;
    batcher->offset_y = viewport.min_y + half_y;
    
    #ifdef DMIR_DEPTH_INT32
    batcher->depth_factor = DMIR_MAX_DEPTH / (frustum.max_depth - frustum.min_depth);
    #else
    batcher->depth_factor = 1.0f;
    #endif
    
    batcher->eye_z = -batcher->scale_c / batcher->scale_z;
    batcher->is_perspective = (batcher->scale_z > 1e-16f) | (batcher->scale_z < -1e-16f);
    batcher->clamp_z = batcher->eye_z + MAX(-batcher->eye_z, 1) * 1e-8f;
    
    batcher->frustum_bounds.min_x = viewport.min_x;
    batcher->frustum_bounds.max_x = viewport.max_x + 1;
    batcher->frustum_bounds.min_y = viewport.min_y;
    batcher->frustum_bounds.max_y = viewport.max_y + 1;
    batcher->frustum_bounds.min_z = MAX(frustum.min_depth, batcher->eye_z);
    batcher->frustum_bounds.max_z = frustum.max_depth;
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
        void* subtrees_start = batcher->subtrees;
        batcher->subtrees_size *= 2;
        batcher->subtrees = realloc(batcher->subtrees, batcher->subtrees_size * sizeof(Subtree));
        batcher->sorted = realloc(batcher->sorted, batcher->subtrees_size * sizeof(Subtree*));
        // Since sorted stores pointers, we need to re-point them to the new subtrees array
        void** sorted = (void**)batcher->sorted;
        void** sorted_end = (void**)(batcher->sorted + batcher->subtrees_count);
        for (; sorted != sorted_end; sorted++) {
            *sorted = (void*)batcher->subtrees + ((*sorted) - subtrees_start);
        }
    }
    Subtree* subtree = batcher->subtrees + batcher->subtrees_count;
    batcher->sorted[batcher->subtrees_count] = subtree;
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
    
    Rect viewport = batcher->api.viewport;
    Bounds frustum_bounds = batcher->frustum_bounds;
    
    SInt viewport_size_x = viewport.max_x - viewport.min_x + 1;
    SInt viewport_size_y = viewport.max_y - viewport.min_y + 1;
    SInt viewport_size_max = MAX(viewport_size_x, viewport_size_y);
    SInt max_subtree_size = CLAMP(batcher->api.split_factor, 0.0f, 1.0f) * viewport_size_max;
    
    uint32_t address = root;
    SInt mask = *PTR_INDEX(octree->mask, address);
    uint32_t child_start = *PTR_INDEX(octree->addr, address);
    
    stack->level = 0;
    stack->affine_id = -1;
    stack->rect = viewport;
    stack->is_behind = (bounds.min_z <= batcher->eye_z);
    
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
            SInt octant = stack->octants[stack->count];
            address = stack->node + octant;
            
            // Copy corner vertices from the parent grid's sub-octant vertices
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
        
        SInt intersects_eye_plane = (bounds.min_z <= batcher->eye_z);
        
        if (intersects_eye_plane) {
            RECT_CLIP(bounds, frustum_bounds);
        }
        
        // Calculate screen-space bounds (in pixels)
        Rect rect;
        RECT_FROM_BOUNDS_F(rect, bounds);
        
        // Calculate node size (in pixels)
        SInt size_x = rect.max_x - rect.min_x;
        SInt size_y = rect.max_y - rect.min_y;
        SInt max_size = MAX(size_x, size_y);
        SInt is_pixel = (max_size == 0);
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
            if (is_occluded_quad(framebuffer, rect, depth)) continue;
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
            
            SInt is_subtree = is_pixel | is_splat;
            is_subtree |= (!intersects_near_plane) & (max_size < max_subtree_size);
            
            if (is_subtree) {
                Subtree* subtree = batcher_add_subtree(batcher);
                subtree->affine_id = stack->affine_id;
                subtree->bounds = bounds;
                cage_from_grid(stack->grid, subtree->cage);
                subtree->address = address;
                subtree->effects = effects;
                subtree->effects.max_level -= stack->level;
                
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
        starting_octant = (~starting_octant) & 7; // opposite (reverse order)
        
        // Push non-empty children on the stack
        stack++;
        stack->node = child_start;
        stack->count = 0;
        stack->level = (stack-1)->level + 1;
        stack->rect = rect;
        stack->is_behind = intersects_eye_plane;
        stack->affine_id = (stack-1)->affine_id;
        
        for (SInt i = 0; i < 8; i++) {
            SInt octant = starting_octant ^ i;
            if ((mask & (1 << octant)) == 0) continue;
            
            stack->octants[stack->count] = octant;
            stack->subnodes[stack->count] = *PTR_INDEX(octree->addr, child_start+octant);
            stack->masks[stack->count] = *PTR_INDEX(octree->mask, child_start+octant);
            stack->count++;
        }
    } while (stack > stack_start);
}

// qsort's callback signature expents int specifically
int subtree_compare(const void *a, const void *b) {
    const Subtree* subtree_a = *(const Subtree**)a;
    const Subtree* subtree_b = *(const Subtree**)b;
    // qsort expects only {-1, 0, 1}, so we can't just cast a difference to int
    // Basic depth sorting seems to work well even for nodes of the same octree?
    // if (subtree_a->affine_id == subtree_b->affine_id) {
    //     // Pointer addresses have the same order as octree traversal
    //     if (subtree_a < subtree_b) return -1;
    //     if (subtree_a > subtree_b) return 1;
    //     return 0;
    // }
    if (subtree_a->bounds.min_z < subtree_b->bounds.min_z) return -1;
    if (subtree_a->bounds.min_z > subtree_b->bounds.min_z) return 1;
    return 0;
}

void dmir_batcher_sort(Batcher* batcher_ptr) {
    BatcherInternal* batcher = (BatcherInternal*)batcher_ptr;
    
    // TODO: use timsort? (probably better for our use-case)
    // Or: topological sort of bucketed/tiled nodes?
    qsort(batcher->sorted, batcher->subtrees_count, sizeof(Subtree*), subtree_compare);
    
    // Current batch is finished and can only be rendered
    batcher->renderable_count = batcher->subtrees_count;
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
    }
    
    return (Renderer*)renderer;
}

void dmir_renderer_free(Renderer* renderer_ptr) {
    RendererInternal* renderer = (RendererInternal*)renderer_ptr;
    
    free(renderer->stack);
    
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
            SInt octant = stack->octants[stack->count];
            address = stack->node + octant;
            
            // Copy corner vertices from the parent grid's sub-octant vertices
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
        
        SInt intersects_eye_plane = (bounds.min_z <= batcher->eye_z);
        
        if (intersects_eye_plane) {
            RECT_CLIP(bounds, frustum_bounds);
        }
        
        // Calculate screen-space bounds (in pixels)
        Rect rect;
        RECT_FROM_BOUNDS_F(rect, bounds);
        
        // Calculate node size (in pixels)
        SInt size_x = rect.max_x - rect.min_x;
        SInt size_y = rect.max_y - rect.min_y;
        SInt max_size = MAX(size_x, size_y);
        SInt is_pixel = (max_size == 0);
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
            
            // Do an occlusion check
            if (is_occluded_quad(framebuffer, rect, depth)) continue;
        }
        
        // Calculate the remaining (non-corner) vertices
        // This is also required for calculating matrix axes and starting octant
        calculate_midpoints(stack->grid, batcher);
        
        // Calculate the starting octant
        SInt starting_octant;
        if (batcher->is_perspective & (bounds.min_z < batcher->eye_z)) {
            starting_octant = calculate_starting_octant_perspective(stack->grid, batcher->eye_z);
        } else {
            starting_octant = calculate_starting_octant(stack->grid);
        }
        starting_octant = (~starting_octant) & 7; // opposite (reverse order)
        
        // Push non-empty children on the stack
        stack++;
        stack->node = child_start;
        stack->count = 0;
        stack->level = (stack-1)->level + 1;
        stack->rect = rect;
        stack->is_behind = intersects_eye_plane;
        
        for (SInt i = 0; i < 8; i++) {
            SInt octant = starting_octant ^ i;
            if ((mask & (1 << octant)) == 0) continue;
            
            stack->octants[stack->count] = octant;
            stack->subnodes[stack->count] = *PTR_INDEX(octree->addr, child_start+octant);
            stack->masks[stack->count] = *PTR_INDEX(octree->mask, child_start+octant);
            stack->count++;
        }
    } while (stack > stack_start);
}

void dmir_renderer_draw(Renderer* renderer_ptr) {
    RendererInternal* renderer = (RendererInternal*)renderer_ptr;
    Framebuffer* framebuffer = renderer->api.framebuffer;
    BatcherInternal* batcher = (BatcherInternal*)renderer->api.batcher;
    
    for (SInt index = 0; index < batcher->renderable_count; index++) {
        render_cage(renderer, batcher, framebuffer, batcher->sorted[index]);
    }
}
