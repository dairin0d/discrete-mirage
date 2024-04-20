// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <stdio.h>

#include "discrete_mirage.h"

#define USE_8X8_STENCIL

#define PTR_OFFSET(array, offset) ((typeof(array))(((char*)(array)) + (offset)))
#define PTR_INDEX(array, index) PTR_OFFSET((array), ((index) * (array##_stride)))

typedef struct Vector2 {
    float x, y;
} Vector2;

typedef struct Vector3 {
    float x, y, z;
} Vector3;

typedef struct ProjectedVertex {
    Vector3 position;
    Vector2 projection;
} ProjectedVertex;

typedef struct Vector3I {
    int32_t x, y, z;
} Vector3I;

const int GRID_SIZE = 3*3*3;

struct DiscreteMirageBuffers discrete_mirage_make_buffers(uint32_t width, uint32_t height) {
    struct DiscreteMirageBuffers buffers = {.width = width, .height = height};
    
    int buffer_size = buffers.width * buffers.height;
    buffers.node = malloc(buffer_size * sizeof(uint32_t));
    buffers.node_stride = sizeof(uint32_t);
    buffers.object = malloc(buffer_size * sizeof(uint32_t));
    buffers.object_stride = sizeof(uint32_t);
    buffers.depth = malloc(buffer_size * sizeof(int32_t));
    buffers.depth_stride = sizeof(int32_t);
    
    int stencil_width = (buffers.width + 7) / 8;
    int stencil_height = (buffers.height + 7) / 8;
    int stencil_size = stencil_width * stencil_height;
    buffers.stencil = malloc(stencil_size * sizeof(uint64_t));
    
    // 32-bit float screen-space coordinates can be halved
    // at most 128 times before they become subpixel
    buffers.grids = malloc(128 * (3*3*3) * sizeof(struct ProjectedVertex));
    
    discrete_mirage_clear_buffers(buffers);
    
    return buffers;
}

void discrete_mirage_free_buffers(struct DiscreteMirageBuffers buffers) {
    free(buffers.node);
    free(buffers.object);
    free(buffers.depth);
    free(buffers.stencil);
    free(buffers.grids);
}

void discrete_mirage_clear_buffers(struct DiscreteMirageBuffers buffers) {
    int buffer_size = buffers.width * buffers.height;
    for (int i = 0; i < buffer_size; i++) {
        *PTR_INDEX(buffers.node, i) = UINT32_MAX;
        *PTR_INDEX(buffers.object, i) = UINT32_MAX;
        *PTR_INDEX(buffers.depth, i) = INT32_MAX;
    }
    
    int stencil_width = (buffers.width + 7) / 8;
    int stencil_height = (buffers.height + 7) / 8;
    int stencil_size = stencil_width * stencil_height;
    for (int i = 0; i < stencil_size; i++) {
        buffers.stencil[i] = 0;
    }
}

#define GRID2(x, y, z) ((x) + 2*((y) + 2*(z)))
#define GRID3(x, y, z) ((x) + 3*((y) + 3*(z)))

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) < (b) ? (b) : (a))

static int calculate_starting_octant(ProjectedVertex* grid) {
    float xx = grid[GRID3(2,1,1)].projection.x - grid[GRID3(1,1,1)].projection.x;
    float xy = grid[GRID3(2,1,1)].projection.y - grid[GRID3(1,1,1)].projection.y;
    float yx = grid[GRID3(1,2,1)].projection.x - grid[GRID3(1,1,1)].projection.x;
    float yy = grid[GRID3(1,2,1)].projection.y - grid[GRID3(1,1,1)].projection.y;
    float zx = grid[GRID3(1,1,2)].projection.x - grid[GRID3(1,1,1)].projection.x;
    float zy = grid[GRID3(1,1,2)].projection.y - grid[GRID3(1,1,1)].projection.y;
    int bitX = (yy * zx <= yx * zy ? 0 : 1);
    int bitY = (zy * xx <= zx * xy ? 0 : 2);
    int bitZ = (xy * yx <= xx * yy ? 0 : 4);
    return bitX | bitY | bitZ;
}

static int calculate_starting_octant_perspective(ProjectedVertex* grid, float eye_z) {
    float px = 0 - grid[GRID3(1,1,1)].position.x;
    float py = 0 - grid[GRID3(1,1,1)].position.y;
    float pz = eye_z - grid[GRID3(1,1,1)].position.z;
    float xx = grid[GRID3(2,1,1)].position.x - grid[GRID3(1,1,1)].position.x;
    float xy = grid[GRID3(2,1,1)].position.y - grid[GRID3(1,1,1)].position.y;
    float xz = grid[GRID3(2,1,1)].position.z - grid[GRID3(1,1,1)].position.z;
    float yx = grid[GRID3(1,2,1)].position.x - grid[GRID3(1,1,1)].position.x;
    float yy = grid[GRID3(1,2,1)].position.y - grid[GRID3(1,1,1)].position.y;
    float yz = grid[GRID3(1,2,1)].position.z - grid[GRID3(1,1,1)].position.z;
    float zx = grid[GRID3(1,1,2)].position.x - grid[GRID3(1,1,1)].position.x;
    float zy = grid[GRID3(1,1,2)].position.y - grid[GRID3(1,1,1)].position.y;
    float zz = grid[GRID3(1,1,2)].position.z - grid[GRID3(1,1,1)].position.z;
    float dotX = (yy*zz - yz*zy)*px + (yz*zx - yx*zz)*py + (yx*zy - yy*zx)*pz;
    float dotY = (zy*xz - zz*xy)*px + (zz*xx - zx*xz)*py + (zx*xy - zy*xx)*pz;
    float dotZ = (xy*yz - xz*yy)*px + (xz*yx - xx*yz)*py + (xx*yy - xy*yx)*pz;
    int bitX = (dotX <= 0 ? 0 : 1);
    int bitY = (dotY <= 0 ? 0 : 2);
    int bitZ = (dotZ <= 0 ? 0 : 4);
    return bitX | bitY | bitZ;
}

static inline void project(ProjectedVertex* vertex,
    float cx, float cy, float scale, float perspective)
{
    scale = scale / (1.0f + perspective * vertex->position.z);
    vertex->projection.x = cx + vertex->position.x * scale;
    vertex->projection.y = cy + vertex->position.y * scale;
}

#define GRID_INIT(grid, cage, x, y, z, cx, cy, scale, perspective) {\
    grid[GRID3(x,y,z)].position = cage[GRID2(x/2,y/2,z/2)];\
    project(&grid[GRID3(x,y,z)], cx, cy, scale, perspective);\
}

#define VERT_MIDPOINT(vmin, vmax, vmid) {\
    vmid.position.x = (vmin.position.x + vmax.position.x) * 0.5f;\
    vmid.position.y = (vmin.position.y + vmax.position.y) * 0.5f;\
    vmid.position.z = (vmin.position.z + vmax.position.z) * 0.5f;\
}
#define GRID_MIDPOINT(grid, x0, y0, z0, x1, y1, z1, cx, cy, scale, perspective) {\
    VERT_MIDPOINT(\
        grid[GRID3(x0,y0,z0)],\
        grid[GRID3(x1,y1,z1)],\
        grid[GRID3((x0+x1)/2,(y0+y1)/2,(z0+z1)/2)]);\
    project(&grid[GRID3((x0+x1)/2,(y0+y1)/2,(z0+z1)/2)], cx, cy, scale, perspective);\
}

static void initialize_grid(ProjectedVertex* grid, Vector3* cage,
    float cx, float cy, float scale, float perspective)
{
    GRID_INIT(grid, cage, 0, 0, 0, cx, cy, scale, perspective);
    GRID_INIT(grid, cage, 2, 0, 0, cx, cy, scale, perspective);
    GRID_INIT(grid, cage, 0, 2, 0, cx, cy, scale, perspective);
    GRID_INIT(grid, cage, 2, 2, 0, cx, cy, scale, perspective);
    GRID_INIT(grid, cage, 0, 0, 2, cx, cy, scale, perspective);
    GRID_INIT(grid, cage, 2, 0, 2, cx, cy, scale, perspective);
    GRID_INIT(grid, cage, 0, 2, 2, cx, cy, scale, perspective);
    GRID_INIT(grid, cage, 2, 2, 2, cx, cy, scale, perspective);
}

static void calculate_midpoints(ProjectedVertex* grid,
    float cx, float cy, float scale, float perspective)
{
    // X edges
    GRID_MIDPOINT(grid, 0,0,0, 2,0,0, cx, cy, scale, perspective);
    GRID_MIDPOINT(grid, 0,2,0, 2,2,0, cx, cy, scale, perspective);
    GRID_MIDPOINT(grid, 0,0,2, 2,0,2, cx, cy, scale, perspective);
    GRID_MIDPOINT(grid, 0,2,2, 2,2,2, cx, cy, scale, perspective);
    // Y edges
    GRID_MIDPOINT(grid, 0,0,0, 0,2,0, cx, cy, scale, perspective);
    GRID_MIDPOINT(grid, 2,0,0, 2,2,0, cx, cy, scale, perspective);
    GRID_MIDPOINT(grid, 0,0,2, 0,2,2, cx, cy, scale, perspective);
    GRID_MIDPOINT(grid, 2,0,2, 2,2,2, cx, cy, scale, perspective);
    // Z edges
    GRID_MIDPOINT(grid, 0,0,0, 0,0,2, cx, cy, scale, perspective);
    GRID_MIDPOINT(grid, 2,0,0, 2,0,2, cx, cy, scale, perspective);
    GRID_MIDPOINT(grid, 0,2,0, 0,2,2, cx, cy, scale, perspective);
    GRID_MIDPOINT(grid, 2,2,0, 2,2,2, cx, cy, scale, perspective);
    // Faces
    GRID_MIDPOINT(grid, 1,0,0, 1,2,0, cx, cy, scale, perspective);
    GRID_MIDPOINT(grid, 0,1,0, 0,1,2, cx, cy, scale, perspective);
    GRID_MIDPOINT(grid, 0,0,1, 2,0,1, cx, cy, scale, perspective);
    GRID_MIDPOINT(grid, 1,0,2, 1,2,2, cx, cy, scale, perspective);
    GRID_MIDPOINT(grid, 2,1,0, 2,1,2, cx, cy, scale, perspective);
    GRID_MIDPOINT(grid, 0,2,1, 2,2,1, cx, cy, scale, perspective);
    // Center
    GRID_MIDPOINT(grid, 1,1,0, 1,1,2, cx, cy, scale, perspective);
}

static void initialize_subgrid(ProjectedVertex* subgrid, ProjectedVertex* grid, int octant) {
    int ox = (octant >> 0) & 1;
    int oy = (octant >> 1) & 1;
    int oz = (octant >> 2) & 1;
    subgrid[GRID3(0,0,0)] = grid[GRID3(0+ox,0+oy,0+oz)];
    subgrid[GRID3(2,0,0)] = grid[GRID3(1+ox,0+oy,0+oz)];
    subgrid[GRID3(0,2,0)] = grid[GRID3(0+ox,1+oy,0+oz)];
    subgrid[GRID3(2,2,0)] = grid[GRID3(1+ox,1+oy,0+oz)];
    subgrid[GRID3(0,0,2)] = grid[GRID3(0+ox,0+oy,1+oz)];
    subgrid[GRID3(2,0,2)] = grid[GRID3(1+ox,0+oy,1+oz)];
    subgrid[GRID3(0,2,2)] = grid[GRID3(0+ox,1+oy,1+oz)];
    subgrid[GRID3(2,2,2)] = grid[GRID3(1+ox,1+oy,1+oz)];
}

static void calculate_screen_bounds(ProjectedVertex* grid, float* result) {
    result[0] = INFINITY;
    result[1] = -INFINITY;
    result[2] = INFINITY;
    result[3] = -INFINITY;
    result[4] = INFINITY;
    result[5] = -INFINITY;
    for (int iz = 0; iz < 3; iz += 2) {
        for (int iy = 0; iy < 3; iy += 2) {
            for (int ix = 0; ix < 3; ix += 2) {
                result[0] = MIN(result[0], grid[GRID3(ix, iy, iz)].projection.x);
                result[1] = MAX(result[1], grid[GRID3(ix, iy, iz)].projection.x);
                result[2] = MIN(result[2], grid[GRID3(ix, iy, iz)].projection.y);
                result[3] = MAX(result[3], grid[GRID3(ix, iy, iz)].projection.y);
                result[4] = MIN(result[4], grid[GRID3(ix, iy, iz)].position.z);
                result[5] = MAX(result[5], grid[GRID3(ix, iy, iz)].position.z);
            }
        }
    }
}

const int SUBPIXEL_BITS = 16;
const int SUBPIXEL_SIZE = 1 << SUBPIXEL_BITS;
const int SUBPIXEL_HALF = SUBPIXEL_SIZE >> 1;

#define VERT_MIDPOINT_I(vmin, vmax, vmid) {\
    vmid.x = (vmin.x + vmax.x) >> 1;\
    vmid.y = (vmin.y + vmax.y) >> 1;\
    vmid.z = (vmin.z + vmax.z) >> 1;\
}
#define GRID_MIDPOINT_I(grid, x0, y0, z0, x1, y1, z1) {\
    VERT_MIDPOINT_I(\
        grid[GRID3(x0,y0,z0)],\
        grid[GRID3(x1,y1,z1)],\
        grid[GRID3((x0+x1)/2,(y0+y1)/2,(z0+z1)/2)]);\
}

static void initialize_grid_i(Vector3I* grid, ProjectedVertex* grid_f, float depth_scale)
{
    for (int i = 0; i < GRID_SIZE; i++) {
        grid[i].x = (int)(grid_f[i].projection.x * SUBPIXEL_SIZE);\
        grid[i].y = (int)(grid_f[i].projection.y * SUBPIXEL_SIZE);\
        grid[i].z = (int)(grid_f[i].position.z * depth_scale);\
    }
}

static void calculate_midpoints_i(Vector3I* grid)
{
    // X edges
    GRID_MIDPOINT_I(grid, 0,0,0, 2,0,0);
    GRID_MIDPOINT_I(grid, 0,2,0, 2,2,0);
    GRID_MIDPOINT_I(grid, 0,0,2, 2,0,2);
    GRID_MIDPOINT_I(grid, 0,2,2, 2,2,2);
    // Y edges
    GRID_MIDPOINT_I(grid, 0,0,0, 0,2,0);
    GRID_MIDPOINT_I(grid, 2,0,0, 2,2,0);
    GRID_MIDPOINT_I(grid, 0,0,2, 0,2,2);
    GRID_MIDPOINT_I(grid, 2,0,2, 2,2,2);
    // Z edges
    GRID_MIDPOINT_I(grid, 0,0,0, 0,0,2);
    GRID_MIDPOINT_I(grid, 2,0,0, 2,0,2);
    GRID_MIDPOINT_I(grid, 0,2,0, 0,2,2);
    GRID_MIDPOINT_I(grid, 2,2,0, 2,2,2);
    // Faces
    GRID_MIDPOINT_I(grid, 1,0,0, 1,2,0);
    GRID_MIDPOINT_I(grid, 0,1,0, 0,1,2);
    GRID_MIDPOINT_I(grid, 0,0,1, 2,0,1);
    GRID_MIDPOINT_I(grid, 1,0,2, 1,2,2);
    GRID_MIDPOINT_I(grid, 2,1,0, 2,1,2);
    GRID_MIDPOINT_I(grid, 0,2,1, 2,2,1);
    // Center
    GRID_MIDPOINT_I(grid, 1,1,0, 1,1,2);
}

static void initialize_subgrid_i(Vector3I* subgrid, Vector3I* grid, int octant) {
    int ox = (octant >> 0) & 1;
    int oy = (octant >> 1) & 1;
    int oz = (octant >> 2) & 1;
    subgrid[GRID3(0,0,0)] = grid[GRID3(0+ox,0+oy,0+oz)];
    subgrid[GRID3(2,0,0)] = grid[GRID3(1+ox,0+oy,0+oz)];
    subgrid[GRID3(0,2,0)] = grid[GRID3(0+ox,1+oy,0+oz)];
    subgrid[GRID3(2,2,0)] = grid[GRID3(1+ox,1+oy,0+oz)];
    subgrid[GRID3(0,0,2)] = grid[GRID3(0+ox,0+oy,1+oz)];
    subgrid[GRID3(2,0,2)] = grid[GRID3(1+ox,0+oy,1+oz)];
    subgrid[GRID3(0,2,2)] = grid[GRID3(0+ox,1+oy,1+oz)];
    subgrid[GRID3(2,2,2)] = grid[GRID3(1+ox,1+oy,1+oz)];
}

static void calculate_screen_bounds_i(Vector3I* grid, int* result) {
    result[0] = INT32_MAX;
    result[1] = INT32_MIN;
    result[2] = INT32_MAX;
    result[3] = INT32_MIN;
    result[4] = INT32_MAX;
    result[5] = INT32_MIN;
    for (int iz = 0; iz < 3; iz += 2) {
        for (int iy = 0; iy < 3; iy += 2) {
            for (int ix = 0; ix < 3; ix += 2) {
                result[0] = MIN(result[0], grid[GRID3(ix, iy, iz)].x);
                result[1] = MAX(result[1], grid[GRID3(ix, iy, iz)].x);
                result[2] = MIN(result[2], grid[GRID3(ix, iy, iz)].y);
                result[3] = MAX(result[3], grid[GRID3(ix, iy, iz)].y);
                result[4] = MIN(result[4], grid[GRID3(ix, iy, iz)].z);
                result[5] = MAX(result[5], grid[GRID3(ix, iy, iz)].z);
            }
        }
    }
}

typedef struct StackItem {
    uint32_t subnodes[8];
    uint8_t masks[8];
    uint8_t octants[8];
    int node;
    int count;
} StackItem;

#ifdef USE_8X8_STENCIL
#define SPLAT_PIXEL(x, y, z, address, obj_id, buffers, stencil_width) {\
    int tx = x >> 3;\
    int ty = y >> 3;\
    int ti = tx + ty * stencil_width;\
    uint64_t tbit = (1ul << ((x & 7) + (y & 7)*8));\
    if ((buffers.stencil[ti] & tbit) == 0) {\
        buffers.stencil[ti] |= tbit;\
        int i = x + y * buffers.width;\
        *PTR_INDEX(buffers.node, i) = address;\
        *PTR_INDEX(buffers.object, i) = obj_id;\
    }\
}
#define OCCLUSION_TEST(xmin, ymin, xmax, ymax, z, buffers, stencil_width, occlusion_passed) {\
    int txmin = xmin >> 3;\
    int tymin = ymin >> 3;\
    int txmax = xmax >> 3;\
    int tymax = ymax >> 3;\
    for (int ty = tymin; ty <= tymax; ty++) {\
        for (int tx = txmin; tx <= txmax; tx++) {\
            int ti = tx + ty * stencil_width;\
            if (buffers.stencil[ti] != UINT64_MAX) goto occlusion_passed;\
        }\
    }\
}
#else
#define SPLAT_PIXEL(x, y, z, address, obj_id, buffers, stencil_width) {\
    int i = x + y * buffers.width;\
    if (*PTR_INDEX(buffers.depth, i) > z) {\
        *PTR_INDEX(buffers.depth, i) = z;\
        *PTR_INDEX(buffers.node, i) = address;\
        *PTR_INDEX(buffers.object, i) = obj_id;\
    }\
}
#define OCCLUSION_TEST(xmin, ymin, xmax, ymax, z, buffers, stencil_width, occlusion_passed) {\
    for (int y = ymin; y <= ymax; y++) {\
        for (int x = xmin; x <= xmax; x++) {\
            int i = x + y * buffers.width;\
            if (*PTR_INDEX(buffers.depth, i) > z) goto occlusion_passed;\
        }\
    }\
}
#endif

static void render_ortho(
    struct DiscreteMirageBuffers buffers,
    struct DiscreteMirageView view,
    struct DiscreteMirageOctree octree,
    struct DiscreteMirageScene scene,
    StackItem* stack,
    Vector3I* grid,
    int obj_id,
    int starting_octant)
{
    int starting_octant_inv = (~starting_octant) & 7;
    
    StackItem* stack_start = stack - 1;
    
    int screen_bounds[6];
    
    int stencil_width = (buffers.width + 7) / 8;
    int stencil_height = (buffers.height + 7) / 8;
    
    do {
        // Pop an entry from the stack
        if (stack->count == 0) {
            stack--;
            grid -= GRID_SIZE;
            continue;
        }
        stack->count--;
        int child_start = stack->subnodes[stack->count];
        int mask = stack->masks[stack->count];
        int octant = stack->octants[stack->count];
        int address = stack->node + octant;
        
        // Copy corner vertices from the parent grid's sub-octant vertices
        initialize_subgrid_i(grid, (grid - GRID_SIZE), octant);
        
        // Find the min/max of the corner vertices
        calculate_screen_bounds_i(grid, screen_bounds);
        
        // Calculate screen-space bounds (in pixels)
        int xmin = (screen_bounds[0]+SUBPIXEL_HALF) >> SUBPIXEL_BITS;
        int xmax = (screen_bounds[1]-SUBPIXEL_HALF) >> SUBPIXEL_BITS;
        int ymin = (screen_bounds[2]+SUBPIXEL_HALF) >> SUBPIXEL_BITS;
        int ymax = (screen_bounds[3]-SUBPIXEL_HALF) >> SUBPIXEL_BITS;
        
        // Calculate node size (in pixels)
        int xsize = xmax - xmin;
        int ysize = ymax - ymin;
        
        // Clamp to viewport
        if (xmin < view.xmin) xmin = view.xmin;
        if (ymin < view.ymin) ymin = view.ymin;
        if (xmax > view.xmax) xmax = view.xmax;
        if (ymax > view.ymax) ymax = view.ymax;
        
        // Skip if not visible
        if ((xmin > xmax) | (ymin > ymax)) continue;
        
        int z = screen_bounds[4];
        
        // Splat if size is 1 pixel
        if ((xsize|ysize) == 0) {
            SPLAT_PIXEL(xmin, ymin, z, address, obj_id, buffers, stencil_width);
            continue;
        }
        
        // Splat if this is a leaf node
        if (!mask) {
            for (int y = ymin; y <= ymax; y++) {
                for (int x = xmin; x <= xmax; x++) {
                    SPLAT_PIXEL(x, y, z, address, obj_id, buffers, stencil_width);
                }
            }
            continue;
        }
        
        // Do an occlusion check
        OCCLUSION_TEST(xmin, ymin, xmax, ymax, z, buffers, stencil_width, occlusion_passed);
        continue;
        occlusion_passed:;
        
        if ((xsize > 1) | (ysize > 1)) {
            // Calculate the remaining (non-corner) vertices
            calculate_midpoints_i(grid);
            
            // Push non-empty children on the stack
            grid += GRID_SIZE;
            stack++;
            stack->node = child_start;
            stack->count = 0;
            
            for (int i = 0; i < 8; i++) {
                int octant = starting_octant ^ i;
                if ((mask & (1 << octant)) == 0) continue;
                
                stack->octants[stack->count] = octant;
                stack->subnodes[stack->count] = *PTR_INDEX(octree.nodes, child_start+octant);
                stack->masks[stack->count] = *PTR_INDEX(octree.masks, child_start+octant);
                stack->count++;
            }
        } else {
            // If node size is 2x2 pixels or less, splat immediately
            Vector3I center;
            Vector3I position;
            VERT_MIDPOINT_I(grid[GRID3(0,0,0)], grid[GRID3(2,2,2)], center);
            
            for (int i = 0; i < 8; i++) {
                int octant = starting_octant_inv ^ i;
                if ((mask & (1 << octant)) == 0) continue;
                
                int ox = (octant << 1) & 2;
                int oy = (octant << 0) & 2;
                int oz = (octant >> 1) & 2;
                VERT_MIDPOINT_I(center, grid[GRID3(ox,oy,oz)], position);
                
                int x = position.x >> SUBPIXEL_BITS;
                int y = position.y >> SUBPIXEL_BITS;
                
                if ((x < view.xmin) | (x > view.xmax) | (y < view.ymin) | (y > view.ymax)) continue;
                
                SPLAT_PIXEL(x, y, position.z, child_start + octant, obj_id, buffers, stencil_width);
            }
        }
    } while (stack > stack_start);
}

int32_t discrete_mirage_render(
    struct DiscreteMirageBuffers buffers,
    struct DiscreteMirageView view,
    struct DiscreteMirageOctree octree,
    struct DiscreteMirageScene scene)
{
    StackItem stack_start[128];
    
    float screen_bounds[6];
    
    int stencil_width = (buffers.width + 7) / 8;
    int stencil_height = (buffers.height + 7) / 8;
    
    float cx = buffers.width * 0.5f;
    float cy = buffers.height * 0.5f;
    float xy_scale = MAX(cx, cy);
    
    float eye_z = -1.0f / view.perspective;
    int is_perspective = (view.perspective > 1e-16f) | (view.perspective < -1e-16f);
    
    float depth_scale = (1 << 24) / view.depth_range;
    
    for (int obj_id = 0; obj_id < scene.object_count; obj_id++) {
        Vector3* cage = (Vector3*)PTR_INDEX(scene.cages, obj_id);
        int address = *PTR_INDEX(scene.roots, obj_id);
        
        int mask = *PTR_INDEX(octree.masks, address);
        uint32_t child_start = *PTR_INDEX(octree.nodes, address);
        
        ProjectedVertex* grid = (ProjectedVertex*)buffers.grids;
        StackItem* stack = stack_start;
        int starting_octant = -1;
        
        do {
            // Pop an entry from the stack
            if (stack == stack_start) {
                initialize_grid(grid, cage, cx, cy, xy_scale, view.perspective);
            } else {
                if (stack->count == 0) {
                    stack--;
                    grid -= GRID_SIZE;
                    continue;
                }
                stack->count--;
                child_start = stack->subnodes[stack->count];
                mask = stack->masks[stack->count];
                int octant = stack->octants[stack->count];
                address = stack->node + octant;
                
                // Copy corner vertices from the parent grid's sub-octant vertices
                initialize_subgrid(grid, (grid - GRID_SIZE), octant);
            }
            
            // Find the min/max of the corner vertices
            calculate_screen_bounds(grid, screen_bounds);
            
            // Check if the node is behind the near plane or intersects it
            if (screen_bounds[4] < 0) {
                if (screen_bounds[5] <= 0) continue;
                if (!mask) continue;
                goto occlusion_passed;
            }
            
            // Calculate screen-space bounds (in pixels)
            int xmin = (int)(screen_bounds[0]+0.5f);
            int xmax = (int)(screen_bounds[1]-0.5f);
            int ymin = (int)(screen_bounds[2]+0.5f);
            int ymax = (int)(screen_bounds[3]-0.5f);
            
            // Calculate node size (in pixels)
            int xsize = xmax - xmin;
            int ysize = ymax - ymin;
            
            // Clamp to viewport
            if (xmin < view.xmin) xmin = view.xmin;
            if (ymin < view.ymin) ymin = view.ymin;
            if (xmax > view.xmax) xmax = view.xmax;
            if (ymax > view.ymax) ymax = view.ymax;
            
            // Skip if not visible
            if ((xmin > xmax) | (ymin > ymax)) continue;
            
            int z = screen_bounds[4];
            
            // Splat if size is 1 pixel
            if ((xsize|ysize) == 0) {
                SPLAT_PIXEL(xmin, ymin, z, address, obj_id, buffers, stencil_width);
                continue;
            }
            
            // Splat if this is a leaf node
            if (!mask) {
                for (int y = ymin; y <= ymax; y++) {
                    for (int x = xmin; x <= xmax; x++) {
                        SPLAT_PIXEL(x, y, z, address, obj_id, buffers, stencil_width);
                    }
                }
                continue;
            }
            
            // Do an occlusion check
            OCCLUSION_TEST(xmin, ymin, xmax, ymax, z, buffers, stencil_width, occlusion_passed);
            continue;
            occlusion_passed:;
            
            // Calculate the remaining (non-corner) vertices
            calculate_midpoints(grid, cx, cy, xy_scale, view.perspective);
            
            // Calculate the starting octant
            if (is_perspective & (screen_bounds[4] < 0)) {
                starting_octant = calculate_starting_octant_perspective(grid, eye_z);
            } else {
                starting_octant = calculate_starting_octant(grid);
            }
            starting_octant = (~starting_octant) & 7; // opposite (reverse order)
            
            // Push non-empty children on the stack
            grid += GRID_SIZE;
            stack++;
            stack->node = child_start;
            stack->count = 0;
            
            for (int i = 0; i < 8; i++) {
                int octant = starting_octant ^ i;
                if ((mask & (1 << octant)) == 0) continue;
                
                stack->octants[stack->count] = octant;
                stack->subnodes[stack->count] = *PTR_INDEX(octree.nodes, child_start+octant);
                stack->masks[stack->count] = *PTR_INDEX(octree.masks, child_start+octant);
                stack->count++;
            }
            
            // Seams/cracks only seem to become noticeable starting somewhere between 128 and 256
            const int ortho_size = 128;
            if ((xsize < ortho_size) & (ysize < ortho_size) & (screen_bounds[4] >= 0)) {
                initialize_grid_i((Vector3I*)grid, grid - GRID_SIZE, depth_scale);
                render_ortho(buffers, view, octree, scene,
                    stack, ((Vector3I*)grid) + GRID_SIZE, obj_id, starting_octant);
                grid -= GRID_SIZE;
                stack--;
            }
        } while (stack > stack_start);
    }
    
    return 0;
}
