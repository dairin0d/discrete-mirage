// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

// Discrete Mirage: A library for rendering voxel models on CPU.
// Features:
// * can render octree-like geometries (octrees, DAGs,
//   procedural volumes defined by distance functions, etc.)
// * front-to-back splatting with occlusion culling
// * supports cage deformations and splat dilation
// * supports several splat shapes: points, quads, squares, circles, cubes

#ifndef DISCRETE_MIRAGE
#define DISCRETE_MIRAGE

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===================================================== //

const char DMIR_VERSION[] = "2.0.0";

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
// (defaults to int_fast32_t / uint_fast32_t)
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
#define DMIR_STENCIL_1D
// #define DMIR_STENCIL_2D

// Whether to use a "local" stencil buffer
#define DMIR_STENCIL_LOCAL

// Whether to clear the self-occlusion stencil buffer
// after rendering each subtree
#define DMIR_CLEAR_SELF_STENCIL

// ===================================================== //

// These options deal with how nodes are drawn / splatted

// Whether to write pixels immediately or to postpone it
// until the end of current subtree's rendering
#define DMIR_USE_SPLAT_DEFERRED

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

// Whether to count the diagnostic stats
#define DMIR_CALCULATE_STATS

// ===================================================== //

typedef uint8_t DMirBool;

#ifdef DMIR_DEPTH_INT32
typedef int32_t DMirDepth;
const int32_t DMIR_MAX_DEPTH = INT32_MAX >> 1;
#else
typedef float DMirDepth;
const float DMIR_MAX_DEPTH = INFINITY;
#endif

typedef uint64_t DMirAddress;

typedef int32_t DMirAffineID;

const int32_t DMIR_LEVEL_BITS = 5;
const int32_t DMIR_LEVEL_MAX = 32;

// Axis traversal orders. Declared as #define instead
// of const to be usable in a switch statement
#define DMIR_XYZ 0
#define DMIR_XZY 1
#define DMIR_YXZ 2
#define DMIR_YZX 3
#define DMIR_ZXY 4
#define DMIR_ZYX 5

typedef struct DMirQueue {
    uint32_t octants;
    uint32_t indices;
} DMirQueue;

// Precomputed lookup tables for each possible combination
// of node child mask and traversal order / mirroring.
// The counts[] and flips[] tables contain plain values
// (number of set bits and mirrored mask, respectively),
// but the rest pack either "maps" (octants[], indices[])
// or "queues" (sparse[], packed[]) of 4-bit blocks.
// For example, a queue of 4, 1, 3 would look like this:
// 0000 0000 0000 0000 0000 1011 1001 1100
// A map {1:0, 3:7, 6:2} would look like this:
// 0000 1010 0000 0000 1111 0000 1000 0000
typedef struct DMirLookups {
    uint8_t* counts; // [mask] -> bit count
    uint32_t* octants; // [mask] -> index to octant
    uint32_t* indices; // [mask] -> octant to index
    DMirQueue* sparse; // [(order << 8) | mask] -> octants & indices
    DMirQueue* packed; // [(order << 8) | mask] -> octants & indices
    uint8_t* flips; // [(flip << 8) | mask] -> flipped mask
} DMirLookups;

// Holds information about the (sub)nodes at a given
// traversal level.
typedef struct DMirTraversal {
    uint8_t mask[8];
    uint64_t node[8];
    uint64_t data[8];
} DMirTraversal;

typedef struct DMirNodeBox {
    uint32_t x, y, z, level;
} DMirNodeBox;

// The "base class" of traversable structures (octrees,
// DAGs, etc.); implementations of the corresponding
// structs must have it as the first field.
// * traverse_start: given the initial references to
//   a node (node_ref) and a voxel (data_ref), populate
//   dst's 1st item with actual information about the node.
// * traverse_next: given the previous level's traversal
//   info and the item index, populate dst's items with
//   the information about the item node's children.
// * evaluate: given the node's bounding box (defined by
//   depth/level and min corner position in that level),
//   calculate the voxel data reference for the node
//   and return a number indicating whether the node
//   should be skipped (-1), splatted (0) or subdivided
//   further (1).
// For procedural volumes, the evaluate callback should
// be used; for everything else, use traverse_start
// and traverse_next.
// The semantics of node references and data references
// are entirely up to the implementation.
typedef struct DMirGeometry {
    DMirBool (*traverse_start)(void* geometry, DMirAddress node_ref, DMirAddress data_ref, DMirTraversal* dst);
    DMirBool (*traverse_next)(void* geometry, DMirTraversal* src, int32_t index, DMirTraversal* dst);
    int32_t (*evaluate)(void* geometry, DMirNodeBox* node_box, DMirAddress* data_ref);
} DMirGeometry;

// A utility struct for general-purpose hierarchy walks.
// * level_limit: limits the depth of traversal (use
//   a negative value to always traverse to leaf nodes)
// * level_max: the max depth/level encountered so far
// * node_count: number of nodes encountered so far
// * leaf_count: number of leaves encountered so far
// * state: optional pointer for custom user data
// * visit: the callback with the user logic
// * node_box: visited node's bounding box
// * data_ref: visited node's voxel data reference
// * is_leaf: whether the visited node is a leaf
// * mask: visited node's child mask
typedef struct DMirVisitor DMirVisitor;
struct DMirVisitor {
    int32_t level_limit;
    uint32_t level_max;
    uint64_t node_count;
    uint64_t leaf_count;
    void* state;
    DMirBool (*visit)(DMirVisitor* visitor);
    DMirNodeBox node_box;
    DMirAddress data_ref;
    DMirBool is_leaf;
    uint8_t mask;
};

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

// max_level: if >= 0, limits the depth of traversal.
// * dilation_abs: dilate each node by this amount of pixels.
// * dilation_rel: dilate each node by a fraction of the root
//   node's cage size (e.g. for sparse point clouds).
// * shape: how the leaf nodes will be rendered.
typedef struct DMirEffects {
    int32_t max_level;
    float dilation_abs;
    float dilation_rel;
    int32_t shape;
} DMirEffects;

// Each batched geometry may be split into one or more
// of these structs (depending on the cage deformation).
// * group: same value that was passed to dmir_batcher_add(...)
//   (this can be useful if multiple geometry instances may
//   belong to / constitute a single "3D object").
// * geometry: reference to the geometry itself.
// * matrix_normal: view-space matrix for calculating normals.
typedef struct DMirAffineInfo {
    uint32_t group;
    DMirGeometry* geometry;
    float matrix_normal[3*3];
} DMirAffineInfo;

// Framebufer's voxel buffer stores this data in each pixel:
// * affine_id: index in the array of affine_infos.
// * voxel_id: voxel indetifier (typically an index in the
//   geometry's data array, though semantics are up to the user).
typedef struct DMirVoxelRef {
    DMirAffineID affine_id;
    DMirAddress voxel_id;
} DMirVoxelRef;

// An enumeration of diagnostic stats for a framebuffer
enum DMirStats {
    DMIR_NODES_BATCHED,
    DMIR_NODES_CAGE,
    DMIR_NODES_ORTHO,
    DMIR_OCCLUSIONS_FAILED,
    DMIR_OCCLUSIONS_PASSED,
    DMIR_SPLATS_LEAF,
    DMIR_SPLATS_1PX,
    DMIR_SPLATS_2PX,
    DMIR_SPLATS_3PX,
    DMIR_SPLATS_4PX,
    DMIR_FRAGMENTS_ADDED,
    DMIR_FRAGMENTS_WRITTEN,
    DMIR_STATS_COUNT
};

// The public (API) part of a framebuffer object.
// None of its fields are supposed to be changed manually.
// * size_x, size_y: buffer's width and height.
// * row_shift: used when DMIR_ROW_POW2 is enabled.
// * stencil-related fields: if doing multi-threaded rendering,
//   work should be split into regions that don't overlap
//   the same stencil tiles.
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
    uint32_t stats[DMIR_STATS_COUNT];
} DMirFramebuffer;

// The public (API) part of a batcher object.
// * viewport: the rectangle (in pixels) to which the camera's
//   view will be rendered. Read-only (don't change it).
// * frustum: the camera frustum parameters (also "read-only").
// * split_factor: the fraction of framebuffer's max dimension
//   (width or height) above which a geometry will be split into
//   separate subtrees that would be sorted by depth.
// * affine_distortion: the amount of distortion (in pixels)
//   below which a subtree is considered affine (non-deformed).
// * ortho_distortion: the amount of distortion (in pixels)
//   below which a subtree is considered orthographic.
// * rect: the bounding rectangle (in pixels) of all the 
//   batched subtrees' projections.
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
// * Startup: make lookups, framebuffer(s), batcher(s), renderer(s)
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
//   5. Read the color data (based on the affine_id
//      and voxel address of each pixel)
//      and write it to your texture or canvas.
// * Cleanup: free framebuffer(s), batcher(s), renderer(s)

// Some additional remarks regarding dmir_batcher_add:
// it adds an instance of geometry to the batch as 1 or more
// DMirAffineInfo entries (this depends on how deformed,
// or non-affine, its cage is). The cage should be an array
// of 8 triplets of (x,y,z) float coordinates, corresponding
// to the cage's corner vertices. The "group" argument can be
// useful if multiple geometries belong to one logical "object".

// The lookup tables can take quite a bit of space (~1.5 MB),
// so in this library they are generated at runtime.
DMirLookups* dmir_lookups_make();
void dmir_lookups_free(DMirLookups* lookups);

DMirFramebuffer* dmir_framebuffer_make(uint32_t size_x, uint32_t size_y);
void dmir_framebuffer_free(DMirFramebuffer* framebuffer);
void dmir_framebuffer_resize(DMirFramebuffer* framebuffer, uint32_t size_x, uint32_t size_y);
void dmir_framebuffer_clear(DMirFramebuffer* framebuffer);

DMirBatcher* dmir_batcher_make(DMirLookups* lookups);
void dmir_batcher_free(DMirBatcher* batcher);
void dmir_batcher_reset(DMirBatcher* batcher, DMirRect viewport, DMirFrustum frustum);
void dmir_batcher_add(DMirBatcher* batcher, DMirFramebuffer* framebuffer,
    uint32_t group, float* cage, DMirGeometry* geometry, DMirAddress root, DMirEffects effects);
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

void dmir_visit(DMirGeometry* geometry, DMirAddress node_ref, DMirVisitor* visitor, DMirBool reverse);

// ===================================================== //

#ifdef __cplusplus
}
#endif

#endif
