// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

// Discrete Mirage: A library for rendering voxel models on CPU.
// Features:
// * can render basic in-RAM octrees of a specific layout
// * front-to-back splatting with occlusion culling
// * supports cage deformations and splat dilation
// * supports several splat shapes: points, quads, squares, circles, cubes
// Limitations:
// * currently has no support for DAGs or out-of-core rendering

#ifndef DISCRETE_MIRAGE
#define DISCRETE_MIRAGE

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===================================================== //

const char DMIR_VERSION[] = "1.2.0";

// ===================================================== //

// These are mainly for testing the performance effects
// of some general things like number representations,
// conditional assignment, multiply vs shift, and such.

// Representation of depth values
#define DMIR_DEPTH_INT32
// #define DMIR_DEPTH_FLOAT

// Representation of screen-space coordinates
// (for fixed-point, defines the number of bits
// reserved for the fractional part)
#define DMIR_COORD_FIXED 16
// #define DMIR_COORD_FLOAT

// Size of integer types used in some calculations
// (defaults to C compiler's int)
// #define DMIR_INT_SIZE 32
// #define DMIR_INT_SIZE 64

// Whether an if statement or a ternary expression
// is used for updating min and max values
// #define DMIR_MINMAX_IF
#define DMIR_MINMAX_TERNARY

// Whether renderbuffer width should be rounded up
// to the next power-of-2 (i.e., whether to use
// shift or multiply to calculate pixel indices)
// #define DMIR_ROW_POW2

// ===================================================== //

// These options relate to orthographic-mode rendering

// Whether to render using orthographic approximation
// or cage subdivision (note that they are mutually
// exclusive in this library and are never used together)
#define DMIR_USE_ORTHO

// Whether to use the default ("parent-on-stack") or
// an alternative ("children-on-stack") method for octree
// traversal during the orthographic-mode rendering
// #define DMIR_ORTHO_TRAVERSE_ALT

// ===================================================== //

// These options deal with occlusion-related things

// Whether to use occlusion (just to check how much
// worse the performance would be without it)
#define DMIR_USE_OCCLUSION

// This option adjusts the childrens' clip rects
// when some rows of the parent's rect are occluded
#define DMIR_SKIP_OCCLUDED_ROWS

// These options enable the use of a stencil buffer
// with the corresponding number of bits in each tile
// #define DMIR_STENCIL_BITS 16
// #define DMIR_STENCIL_BITS 32
#define DMIR_STENCIL_BITS 64

// Whether a stencil tile should be treated
// as a 1D row of bits or a 2D bitmap
// #define DMIR_STENCIL_1D
// #define DMIR_STENCIL_2D

// Whether to do accurate stencil checks (using a node's
// exact clip rect) or simply check if a tile is non-full
#define DMIR_STENCIL_EXACT
// #define DMIR_STENCIL_COARSE

// Whether to clear the self-occlusion stencil buffer
// after rendering each subtree
#define DMIR_CLEAR_SELF_STENCIL

// ===================================================== //

// These options deal with how nodes are drawn / splatted

// Whether to write pixels immediately or to postpone it
// until the end of current subtree's rendering
#define DMIR_USE_SPLAT_DEFERRED

// Draw a node as a pixel when reaching 1x1 px node size
// (if disabled, general rect-drawing code will be used
// for nodes of 1x1 px size)
#define DMIR_USE_SPLAT_PIXEL

// Draw sub-nodes immediately as pixels at the projected
// node centers, when reaching <= 2x2 px node size
// #define DMIR_USE_BLIT_AT_2X2

// This determines the size (resolution) of octant and
// sub-octant maps (specified as a power-of-2)
// Note: maps also require DMIR_COORD_FIXED to be used
#define DMIR_MAP_PRECISION 6

// Use a precalculated octant map at <= 2x2 px node size
#define DMIR_USE_MAP_2

// Use a precalculated sub-octant map at <= 3x3 px node size
// (this is a hybrid between 2x2 and 4x4 cases: it uses
// sub-octant map for better shape precision, but still
// takes voxel addresses from the same octants as 2x2)
#define DMIR_USE_MAP_3

// Use a precalculated sub-octant map at <= 4x4 px node size
#define DMIR_USE_MAP_4

// This imitates the effect of "drawing subnodes as pixels"
// by enlarging their extents to always cover a pixel
#define DMIR_MAP_ENSURE_PIXEL

// ===================================================== //

// Enable this if you aren't sure that your octree(s)
// contain valid node addresses everywhere
#define DMIR_VALIDATE_ADDRESSES

// ===================================================== //

typedef uint8_t DMirBool;

#ifdef DMIR_DEPTH_INT32
typedef int32_t DMirDepth;
const int32_t DMIR_MAX_DEPTH = INT32_MAX >> 1;
#else
typedef float DMirDepth;
const float DMIR_MAX_DEPTH = INFINITY;
#endif

// Note: values are treated as inclusive [min, max] range
typedef struct DMirRect {
    int32_t min_x;
    int32_t min_y;
    int32_t max_x;
    int32_t max_y;
} DMirRect;

// min_depth and max_depth are the near and far planes.
// A quad of size focal_extent*2 at distance focal_depth
// from the camera will always occupy the whole viewport
// at all values of perspective. perspective = 0.0 would
// result in fully orthographic projection, 1.0 would
// result in fully perspective projection, and other
// values in this range would interpolate between them.
// Note that negative values are not accounted for, and
// would probably result in undefined behavior.
// The offset fields can be used for view jittering.
typedef struct DMirFrustum {
    float min_depth;
    float max_depth;
    float focal_extent;
    float focal_depth;
    float perspective;
    float offset_x;
    float offset_y;
} DMirFrustum;

// addr: address (index) of a node's first child.
// mask: node's octant mask (zero mask indicates a leaf).
// data: node's voxel data.
// *_stride: offset (in bytes) between each item (this way,
// addr/mask/data can be stored as separate arrays or as
// one interleaved array).
// count: number of nodes (used to check if node addresses
// stay within array bounds; see DMIR_VALIDATE_ADDRESSES).
// max_level: if >= 0, limits the depth of octree traversal.
// is_packed: whether the octree nodes are packed (empty
// children are not stored) or sparse (always 8 children).
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

// max_level: if >= 0, limits the depth of octree traversal.
// dilation_abs: dilate each node by this amount of pixels.
// dilation_rel: dilate each node by a fraction of the root
// octree's cage size (e.g. for sparse point clouds).
// shape: how the leaf nodes will be rendered.
typedef struct DMirEffects {
    int32_t max_level;
    float dilation_abs;
    float dilation_rel;
    int32_t shape;
} DMirEffects;

// Each batched octree may be split into one or more
// of these structs (depending on the cage deformation).
// group: same value that was passed to dmir_batcher_add(...)
// (this can be useful if multiple octree instances may
// belong to / constitute a single "3D object").
// octree: reference to the octree itself.
// matrix_normal: view-space matrix for calculating normals.
typedef struct DMirAffineInfo {
    uint32_t group;
    DMirOctree* octree;
    float matrix_normal[3*3];
} DMirAffineInfo;

// Framebufer's voxel buffer stores this data in each pixel:
// affine_id: index in the array of affine_infos.
// address: node index in the octree's data array.
typedef struct DMirVoxelRef {
    int32_t affine_id;
    uint32_t address;
} DMirVoxelRef;

// The public (API) part of a framebuffer object.
// None of its fields are supposed to be changed manually.
// size_x, size_y: buffer's width and height.
// row_shift: used when DMIR_ROW_POW2 is enabled.
// stencil-related fields: if doing multi-threaded rendering,
// work should be split into regions that don't overlap
// the same stencil tiles.
typedef struct DMirFramebuffer {
    DMirDepth* depth;
    DMirVoxelRef* voxel;
    uint32_t size_x;
    uint32_t size_y;
    uint32_t row_shift;
    uint32_t stencil_size_x;
    uint32_t stencil_size_y;
    uint32_t stencil_count_x;
    uint32_t stencil_count_y;
} DMirFramebuffer;

// The public (API) part of a batcher object.
// viewport: the rectangle (in pixels) to which the camera's
// view will be rendered. Read-only (don't change it).
// frustum: the camera frustum parameters (also "read-only").
// split_factor: the fraction of framebuffer's max dimension
// (width or height) above which octrees will be split into
// separate subtrees that would be sorted by depth.
// affine_distortion: the amount of distortion (in pixels)
// below which a subtree is considered affine (non-deformed).
// ortho_distortion: the amount of distortion (in pixels)
// below which a subtree is considered orthographic.
// rect: the bounding rectangle (in pixels) of all the 
// batched subtrees' projections.
typedef struct DMirBatcher {
    DMirRect viewport;
    DMirFrustum frustum;
    float split_factor;
    float affine_distortion;
    float ortho_distortion;
    DMirRect rect;
} DMirBatcher;

// The public (API) part of a renderer object.
// framebuffer and batcher are references to the
// corresponding objects, and rect is the actual
// rectangle to which rendering will be limited
// (e.g. can be smaller than viewport in case of
// splitting the work between multiple renderers).
typedef struct DMirRenderer {
    DMirFramebuffer* framebuffer;
    DMirBatcher* batcher;
    DMirRect rect;
} DMirRenderer;

const int32_t DMIR_SHAPE_POINT = 0;
const int32_t DMIR_SHAPE_RECT = 1;
const int32_t DMIR_SHAPE_SQUARE = 2;
const int32_t DMIR_SHAPE_CIRCLE = 3;
const int32_t DMIR_SHAPE_CUBE = 4;

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
    .offset_x = 0,
    .offset_y = 0,
};

// ===================================================== //

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

// ===================================================== //

// Most of the API functions below should be pretty
// self-explanatory. Here's an overall idea of how
// they are supposed to be used:
// * Startup: make framebuffer(s), batcher(s), renderer(s)
// * Rendering a frame:
//   1. dmir_framebuffer_clear(...)
//   2. dmir_batcher_reset(...)
//   3. for each batch that you want to render:
//      3.1. for each object in the batch:
//           * dmir_batcher_add(...)
//      3.2. dmir_batcher_sort(...)
//      3.3. for each renderer of this batch:
//           * dmir_renderer_draw(...)
//   4. dmir_batcher_affine_get(...)
//   5. Read the color data from the octrees (based on
//      the affine_id and voxel address of each pixel)
//      and write it to your texture or canvas.
// * Cleanup: free framebuffer(s), batcher(s), renderer(s)

// Some additional remarks regarding dmir_batcher_add:
// it adds an instance of octree to the batch as 1 or more
// DMirAffineInfo entries (this depends on how deformed,
// or non-affine, its cage is). The cage should be an array
// of 8 triplets of (x,y,z) float coordinates, corresponding
// to the cage's corner vertices. The "group" argument can be
// useful if multiple octrees belong to one logical "object".

DMirFramebuffer* dmir_framebuffer_make(uint32_t size_x, uint32_t size_y);
void dmir_framebuffer_free(DMirFramebuffer* framebuffer);
void dmir_framebuffer_resize(DMirFramebuffer* framebuffer, uint32_t size_x, uint32_t size_y);
void dmir_framebuffer_clear(DMirFramebuffer* framebuffer);

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

// ===================================================== //

// Also, some utility functions:

// Converts a view-space z coordinate to Depth value
DMirDepth dmir_z_to_depth(DMirBatcher* batcher, float z);

// Checks whether a screen-space quad at a given depth is fully occluded
DMirBool dmir_is_occluded_quad(DMirFramebuffer* framebuffer, DMirRect rect, DMirDepth depth);

// ===================================================== //

#ifdef __cplusplus
}
#endif

#endif
