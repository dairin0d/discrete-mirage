# Notes

* [Version "proto-1.0.0"](#version-proto-100)
  * [About](#about)
  * [Architecture](#architecture)
  * [Observations](#observations)
* [Version "proto-1.1.1"](#version-proto-111)
* [Version "proto-1.1.2"](#version-proto-112)
* [Version "1.2.0"](#version-120)
* [Version "2.0.0"](#version-200)

## Version "proto-1.0.0"

### About

Since my last experiment with [octree splatting in C#](https://github.com/dairin0d/OctreeSplatting), I got inspired to try some additional ideas -- this time, biting the bullet and taking a stab at writing it in C.

The result ended up being this little project that's currently still more of an experiment than anything else. The goal of the first prototype version was to implement most of the ideas I tried so far in a way that would allow them to be tested against (or in combination with) each other -- both to have a solidly working starting point and to determine how those different ideas impact performance.

Another goal I had in mind was making a sort of "reference implementation" of a reasonably fast CPU-based microvoxel renderer, for demonstrational / educational purposes (so that people curious about the subject could take a look and get a reasonable idea what it might involve). I'm not quite sure this goal is currently achieved, both due to the experimental nature of this version, and because I don't know much about the best practices of writing well-structured C code. Oh well :-)

For the sake of completeness, I tried to organize the library's API in a way that would be handy for using it from other applications, though I can't say I expect it to have much practical utility. For one, most real projects would be better off doing rendering on GPU, and also because this version has no support for DAGs / voxel compression (such as [this](https://www.cse.chalmers.se/~uffe/dolonius2017i3d.pdf "Compressing Color Data for Voxelized Surface Geometry") or [this](https://diglib.eg.org/items/d34c8b98-8291-4b7b-84cc-2bad7e162587 "Interactively Modifying Compressed Sparse Voxel Representations")) or "out-of-core" rendering of huge datasets that don't fit into RAM.

### Architecture

At a high level, the library's architecture can be summarized as follows:
* Rendering is done via 3 types of objects:
  * **Framebuffer:** contains voxel, color, depth and stencil data for each pixel.
  * **Batcher:** turns a set of input octree instances into a depth-sorted "batch" of [affine](https://en.wikipedia.org/wiki/Affine_transformation) (within a given tolerance) subtrees which overlap the view frustum. In case you know that some instances are much more likely to be occluders and others are more likely to be occluded, it might make sense to split the rendering of the scene into multiple passes.
  * **Renderer:** actually renders the batched subtrees to the framebuffer (or a portion of it). In case of parallel processing, each thread would need a separate renderer object.
* Rendering loop:
  * Clear the framebuffer contents
  * Reset/re-initialize the batcher state
  * For each set of instances in a rendering pass:
    * For each instance in the set:
      * Add the instance to the batcher
    * Sort the batched subtrees
    * For each renderer:
      * Render the batch to (a portion of) the framebuffer
  * Calculate final colors and/or other data, based on:
    * Batcher's accumulated list of subtrees
    * Framebuffer contents
* Octree traversal happens in multiple places:
  * When batching an instance (uses cage subdivision); descends until a given node is a leaf or a sufficiently affine subtree with projected size below a certain limit.
  * When rendering a batched subtree (uses cage subdivision); descends until a given node is a leaf or a sufficiently orthogonal (in screen-space) subtree with projected size below a certain limit.
  * When rendering "orthogonal" subtrees (uses pre-calculated offsets and extents that are halved on each level).
  * Front-to-back ordering of octants is done by calculating the starting octant and then iterating over a queue of octants/indices pre-computed for each possible value of a child mask.
* Occlusion checks are done using depth and/or stencil tests:
  * Stencil buffer is represented by tiles containing:
    * Self-occlusion bitmask
    * Scene occlusion bitmask
    * Max depth of the filled pixels in the tile
  * For a given quad, a tile is considered occluded if:
    * Self-occlusion bitmask has only zero bits in the quad's rectangle
    * Or the same is true for scene occlusion bitmask, but only if quad's depth is further than max depth
  * If a whole framebuffer row is found to be occluded, node's boundary rect is updated so that its children wouldn't need to check the same row again
  * While rendering a subtree, only the depth buffer and the self-occlusion bitmasks are updated
  * After a subtree is rendered, the self-occlusion bitmasks are merged into the scene occlusion bitmasks (and cleared for the next subtree), and tiles' max depths are updated
* Orthogonal subtree rendering is additionally accelerated via maps:
  * Octant and sub-octant maps (for X and Y axes) are calculated before starting the subtree traversal
  * They contain masks of children (8 bits) or grandchildren (64 bits) overlapping a given pixel of the map
  * When a node's projected size becomes less than 2x2 or 4x4:
    * For each pixel in the projected rect, the corresponding map is sampled
    * The sampled masks are combined with the node's mask, which gives a filtered set of children or grandchildren that overlap a given pixel
    * If the set is empty, the pixel can be skipped
    * Otherwise, the closest octant / sub-octant is determined via look-up tables and/or binary search

### Observations

In case you've read the [overview](https://github.com/dairin0d/OctreeSplatting/blob/main/Notes/Overview.md) I wrote about my C# experiments (or simply are curious), below I list some thoughts and observations I've made over the course of developing this prototype. Though take them with a grain of salt, since I was simply measuring FPS rather than doing any genuine profiling, and a programmer who's well-experienced in C / advanced optimizations might possibly achieve some different results.

Without further ado, here they are:

**C vs C#:** the C renderer is not quite an exact copy of the C# one (for a true apples-to-apples comparison, I'd probably need to backport this library to C#), but their core logic is still very similar. On my current laptop, in the same test scene I observed 33 ms in the Godot C# build, and 23 ms in the C demo of this library. Though that doesn't necessarily mean that C is inherently much faster, since I saw a similar disparity between .NET and Mono versions -- so perhaps if Godot was using .NET runtime, the performance quite plausibly might have been comparable to C. That said, there are still some curious qualitative differences:
- **If vs ternary:** ternary assignment seems a bit faster than conditional assignment in C (in contrast to C#, where it was the other way around). Though perhaps it depends on a particular CPU (for C#, I was doing such tests quite a while ago, on a different machine).
- **Octree storage sparsity:** doesn't seem to matter much in C (interleaved-data, separate-data and compact/packed octrees all performed about the same). In C#, the packed version was slightly faster.
- **Pixel blitting vs 2x2 map:** 2x2 pixel-blitting is about the same as 2x2 map in C. In C#, 2x2 map was a clear winner.
- **Stencil size:** in C, various stencil sizes (ushort, uint, ulong) and dimensions (1D, 2D) all seem to perform comparably to each other. In C#, the 64-bit (8x8) stencil version was performing the best.
- **Accessing the color data:** writing color immediately, writing color as a post-process, and not using color - all seem to have the same performance in C. On the other hand, accessing the color data in C# was taking a noticeable chunk of frame time (though currently I can't rule out that it might be a peculiarity of Godot's version of Mono).
- **Deferred vs immediate writing:** in C, there appears to be no difference between splatting pixels immediately or postponing it until after the octree traversal. In C#, deferred writing was showing a better performance.

**Int vs float:** using ints/fixed-point is a bit faster than floats, though not by much on a modern CPU. However, with int representation it's easier to implement algorithms involving positive/negative exponents of two.

**Traversal approach:** parent-on-stack traversal seems a bit faster than children-on-stack traversal.

**Skipping occluded rows:** seems to help a bit, though not by much.

**Depth vs coarse stencil vs accurate stencil:** when done properly/accurately, using stencil is faster than just the depth (especially for recursive cubes). BUT: a coarse stencil check is worse than just using depth.

**Octree splitting & overdraw:** at least in the test scene, making split_factor large (i.e. essentially sorting only root objects themselves, rather than smaller subtrees) is faster, which is likely due to a larger subtree making a better advantage of self-occlusion (both when using stencil and when relying only on depth). Which implies that avoiding self-overdraw might be more important in such scenes than overdrawing other objects.

**Perspective then & now:** in the C# version, perspective was noticeably slower than orthographic. I suspect this was caused by a faulty logic in cage subdivision and node bounds calculation when nodes intersected the near plane, and also because orthographic subtrees weren't reusing stencil when they could (i.e. if they had affine/non-deformed cages). With a properly implemented logic, performance in perspective is more comparable to orthographic performance, though it yet remains to be seen if the same will hold after fixing the logic in the C# project.

**The problem of ["seams"](https://github.com/dairin0d/OctreeSplatting/blob/main/Notes/Overview.md#return-of-the-samurai):** when I implemented dilation more properly, it actually helped. Also, 2x2 "pixel blitting" does not seem to suffer from such a problem either. Also, imitating "pixel-blitting" in map seems to improve the performance a bit (probably due to a higher chance of occluding other nodes).

**Reconstructing voxel positions:** by storing each affine subtree, it's in principle possible to reconstruct normals / cage-local coordinates / motion vectors / etc. even for deformed-cage models. I only implemented the normal matrix calculation for now, though my test models don't have any normal data, so I don't know if it actually works as intended. But in theory it should =)

**Abandoned ideas:** during my experiments, I also had a few ideas that turned out to be dead-ends, but I'll mention them here just in case:
- **Storing the X-map samples:** theoretically, this saves on a bit of computation in each row of "sub-octant map" loop, but it had no measurable effect on performance.
- [**4x4 blit sequence:**](https://github.com/dairin0d/OctreeSplatting/blob/feature/blit-sequence/Godot/OctreeSplatting/OctreeSplatting/OctreeRenderer.cs#L518) if a node's final rectangle occupies a size of 4x4 pixels or less, it's feasible to process only non-occluded pixels (using a pre-computed look-up table), based on the contents of the stencil buffer. However, calculating the lookup key for arbitrary node positions has an overhead of its own, and in practice it doesn't seem to save enough computation to provide any performance improvements.
- **Topological sorting:** if octrees are split into smaller subtrees, I was thinking that maybe they could be sorted in a way that would allow subtrees of the same octree to still statistically benefit from shared self-occlusion bitmasks, while also being drawn before/after the subtrees of other octrees to account for depth ordering. However, after more thinking, I realized that a "best" order of subtree rendering is not really possible to determine without knowing what they actually contain, so I stopped pursuing that idea and just implemented a depth-based linked-list merge sort instead :-)

## Version "proto-1.1.1"

This version implements some additional bugfixes and improvements in the renderer logic, as well as a couple of extra features:

* A rudimentary "temporal antialiasing" / "motion blur" example (in the demo)
* Support for different splat shapes (point, rectangle, square, circle, cube)

## Version "proto-1.1.2"

This version fixes a few bugs I encountered on Windows that I didn't run into when developing the original version on Linux. Also, I switched the demo from SDL2 to RGFW (single-header window abstraction library). Hopefully, now it would be easier for other people to try out this project :-)

## Version "1.2.0"

To simplify further development, some options were removed:
* Color splatting (now the library is fully agnostic of the data stored in voxels)
* Alternative ("children-on-stack") octree traversal
* Support for splatting in cage-subdivision mode
* Coarse stencil checks (now they are always exact)

Changes introduced in this version:
* Switched from compiler's default (u)int to (u)int_fast32_t when no specific int size is defined
* Added some diagnostic stats (basically, how many nodes of various types were processed)
* Implemented a variant of "local stencil buffer" trick mentioned in PND3D notes:
  * If a stencil tile uses N bits, the "local buffer" is N rows of N bits (N-bit integers)
  * When node size (in pixels) is less than N, rendering switches to "local" mode:
    * Corresponding parts of the "main" stencil buffer get copied to the "local" one
    * "Local" stencil buffer is rather small (cache coherency is more likely)
    * In the "local" case, occlusion test is much simpler, and we can skip the boundary checks
  * With 1D stencil tiles, using "local" mode is a bit faster than otherwise (~15% in my measurements), but 2D stencil tiles apparently add enough overhead to actually make "local" mode slower
  * In practice, "local" stencil buffer primarily speeds up the occlusion test, but the majority of rendering time is still dominated by processing small nodes that did not get occluded

## Version "2.0.0"

To support more than just basic octrees, I had to reorganize some of the APIs and internal workings.
Now the library should (in principle) be able to render any octree-like hierarchical volume, if supplied with appropriate traversal callbacks. The performance of the general implementation is somewhat lower than the octree-only one, but it's arguably more useful from the utility perspective.

In the demo, the following examples are implemented:
* Basic octrees
* Procedural volumes (via distance fields)
* SSVDAGs (symmetry-aware sparse voxel directed acyclic graphs)

Though please note that these examples are simply for demonstration purposes and do not actually correspond to any established file formats. Still, now that the algorithm is generalized, supporting additional variants (such as streaming/out-of-core octrees and compressed voxel data) should be at least feasible :-)
