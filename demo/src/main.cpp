// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

// Demo controls:
// * Left mouse button (drag) - orbit the camera
// * Mouse wheel - zoom in/out
// * W, S - move forward/backward
// * A, D - move left/right
// * R, F - move up/down
// * Ctrl - use faster movement speed
// * Shift - use slower movement speed
// * Space - Switch between perspective and orthographic
// * < and > - change perspective FOV
// * { and } - increase/decrease number of threads
// * - and + - change the max octree descent level
// * Tab - depth visualization mode
// * F1 - render leaf nodes as points
// * F2 - render leaf nodes as rectangles
// * F3 - render leaf nodes as squares
// * F4 - render leaf nodes as circles
// * F5 - render leaf nodes as cubes
// * F12 - toggle "antialiasing/motion-blur" mode
// * Esc - quit

#include <string>
#include <stdint.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <thread>

#define RGFW_BUFFER
#define RGFW_IMPLEMENTATION
#include <RGFW.h>

#include "math_utils.hpp"

#include <discrete_mirage.h>

#define PTR_OFFSET(array, offset) ((typeof(array))(((char*)(array)) + (offset)))
#define PTR_INDEX(array, index) PTR_OFFSET((array), ((index) * (array##_stride)))

int64_t get_time_ms() {
    return RGFW_getTimeNS() / 1000000;
}

struct RGB24 {
    uint8_t r, g, b;
};

struct RGBA32 {
    uint8_t r, g, b, a;
};

struct Object3D {
    DMirOctree* octree;
    int root;
    DMirEffects effects;
    
    bool hide;
    
    struct vec3 position;
    struct quat rotation;
    struct vec3 scale;
    struct vec3 cage[8];
};

struct ProgramState {
    RGFW_window* window;
    int last_mouse_x;
    int last_mouse_y;
    
    #ifndef RGFW_BUFFER
    GLuint gl_texture;
    #endif
    
    std::vector<RGBA32> render_buffer;
    
    DMirFramebuffer* dmir_framebuffer;
    DMirBatcher* dmir_batcher;
    std::vector<DMirRenderer*> dmir_renderers;
    
    int thread_case;
    int thread_count;
    int screen_width;
    int screen_height;
    bool is_running;
    
    int frame_count;
    
    struct vec3 cam_pos;
    struct vec3 cam_rot;
    struct vec3 cam_scl;
    int cam_zoom;
    int cam_zoom_fov;
    bool is_cam_orbiting;
    bool is_depth_mode;
    
    DMirRect viewport;
    DMirFrustum frustum;
    
    int max_level;
    int splat_shape;
    
    std::vector<Object3D*> objects;
    std::vector<DMirOctree*> octrees;
    
    bool use_accumulation;
    int accum_count;
    int accum_shift;
    int accum_index;
    std::vector<struct vec2> accum_offsets;
    std::vector<RGB24*> accum_buffers;
    int* accum_weights_lin;
    int* accum_weights_exp;
};

const int OCTREE_LOAD_RAW = 0;
const int OCTREE_LOAD_SPLIT = 1;
const int OCTREE_LOAD_PACKED = 2;

// int octree_load_mode = OCTREE_LOAD_RAW;
// int octree_load_mode = OCTREE_LOAD_SPLIT;
int octree_load_mode = OCTREE_LOAD_PACKED;

static void read_file(std::string path, int* file_size, char** data) {
    file_size[0] = -1;
    data[0] = nullptr;
    
    std::ifstream file(path, std::ios::binary);
    
    if (!file) {
        std::cerr << "Failed to open file: " << path << std::endl;
        return;
    }
    
    file.seekg(0, std::ios::end);
    file_size[0] = file.tellg();
    file.seekg(0, std::ios::beg);
    
    if ((file_size[0] > 0) && (file_size[0] < UINT32_MAX)) {
        data[0] = new char[file_size[0]];
        file.read(data[0], (std::streamsize)file_size[0]);
    } else {
        std::cerr << "Failed to seek or file is too big: " << path << std::endl;
    }
    
    file.close();
}

DMirOctree* load_octree(std::string path, int mode) {
    int file_size = 0;
    char* file_data = nullptr;
    read_file(path, &file_size, &file_data);
    
    // This demo expects each node layout to be:
    // uint32 address + uint8 mask + uint8*3 rgb
    const int node_size = 8;
    
    if ((file_data == nullptr) || (file_size < 8*node_size)) return nullptr;
    
    DMirOctree* octree = new DMirOctree();
    octree->max_level = -1;
    octree->is_packed = false;
    octree->count = file_size / node_size;
    octree->addr = (uint32_t*)file_data;
    octree->mask = ((uint8_t*)file_data) + 4;
    octree->data = ((uint8_t*)file_data) + 5;
    octree->addr_stride = 8;
    octree->mask_stride = 8;
    octree->data_stride = 8;
    
    if (mode != OCTREE_LOAD_RAW) {
        char* new_data = new char[octree->count * (4 + 1 + 3)];
        uint32_t* addr = (uint32_t*)new_data;
        uint8_t* mask = (uint8_t*)(((char*)addr) + octree->count*4);
        RGB24* color = (RGB24*)(mask + octree->count*1);
        
        if (mode == OCTREE_LOAD_PACKED) {
            octree->is_packed = true;
            
            int count = 1;
            addr[0] = 0;
            
            for (int index = 0; index < count; index++) {
                auto node_address = *PTR_INDEX(octree->addr, addr[index]);
                auto node_mask = *PTR_INDEX(octree->mask, addr[index]);
                auto node_color = *((RGB24*)PTR_INDEX(octree->data, addr[index]));
                addr[index] = count;
                mask[index] = node_mask;
                color[index] = node_color;
                
                for (int octant = 0; octant < 8; octant++) {
                    if ((node_mask & (1 << octant)) == 0) continue;
                    
                    addr[count] = node_address + octant;
                    count++;
                    
                    if (count > octree->count) {
                        // Recursion detected, can't proceed
                        delete new_data;
                        delete file_data;
                        delete octree;
                        return nullptr;
                    }
                }
            }
        } else {
            for (int i = 0; i < octree->count; i++) {
                addr[i] = *PTR_INDEX(octree->addr, i);
                mask[i] = *PTR_INDEX(octree->mask, i);
                color[i] = *((RGB24*)PTR_INDEX(octree->data, i));
            }
        }
        
        delete file_data;
        
        octree->addr = addr;
        octree->mask = mask;
        octree->data = (uint8_t*)color;
        octree->addr_stride = 4;
        octree->mask_stride = 1;
        octree->data_stride = 3;
    }
    
    return octree;
}

DMirOctree* make_recursive_cube(int r, int g, int b) {
    uint8_t mask_color[] = {255, (uint8_t)r, (uint8_t)g, (uint8_t)b};
    uint32_t* octree_data = (uint32_t*)(new uint8_t[8 * 8]);
    for (int node_index = 0; node_index < 8; node_index++) {
        octree_data[node_index*2+0] = 0;
        octree_data[node_index*2+1] = ((uint32_t*)mask_color)[0];
    }
    
    DMirOctree* octree = new DMirOctree();
    octree->max_level = -1;
    octree->is_packed = false;
    octree->count = 8;
    octree->addr = octree_data;
    octree->mask = ((uint8_t*)octree_data) + 4;
    octree->data = ((uint8_t*)octree_data) + 5;
    octree->addr_stride = 8;
    octree->mask_stride = 8;
    octree->data_stride = 8;
    return octree;
}

void delete_octree(DMirOctree* octree) {
    delete octree->addr;
    delete octree;
}

struct mat4 calculate_projection_matrix(ProgramState* state) {
    const float zoom_factor = 0.125f;
    float zoom_scale = pow(2.0f, -state->cam_zoom * zoom_factor);
    float zoom_scale_fov = pow(2.0f, -state->cam_zoom_fov * zoom_factor);
    
    float distance_persp = zoom_scale * zoom_scale_fov;
    float distance_ortho = (state->frustum.max_depth - state->frustum.min_depth) * 0.5f;
    state->frustum.focal_depth = lerp(distance_ortho, distance_persp, state->frustum.perspective);
    state->frustum.focal_extent = zoom_scale;
    
    auto view_matrix = trs_matrix(state->cam_pos, state->cam_rot, state->cam_scl);
    auto view_z_axis = get_matrix_vec3(&view_matrix, 2);
    auto offset_axis = svec3_multiply_f(view_z_axis, -state->frustum.focal_depth);
    psmat4_translate(&view_matrix, &view_matrix, &offset_axis);
    
    return smat4_inverse(view_matrix);
}

void create_scene(ProgramState* state, DMirOctree* file_octree) {
    if (file_octree != nullptr) {
        state->octrees.push_back(file_octree);
    }
    
    const float grid_offset = 1.2f;
    const int grid_extent = 2;
    for (int gz = grid_extent; gz >= -grid_extent; gz--) {
        for (int gx = grid_extent; gx >= -grid_extent; gx--) {
            auto octree = file_octree;
            if (file_octree == nullptr) {
                octree = make_recursive_cube(128+gx*32, 255, 128+gz*32);
                state->octrees.push_back(octree);
            }
            
            auto object3d = new Object3D();
            object3d->octree = octree;
            object3d->root = 0;
            object3d->hide = false;
            object3d->effects = {.max_level = -1, .dilation_abs = 0, .dilation_rel = 0};
            object3d->position = svec3(gx * grid_offset, 0, gz * grid_offset);
            object3d->rotation = squat_null();
            object3d->scale = svec3(1, 1, 1);
            for (int octant = 0; octant < 8; octant++) {
                object3d->cage[octant].x = (((octant >> 0) & 1) * 2) - 1;
                object3d->cage[octant].y = (((octant >> 1) & 1) * 2) - 1;
                object3d->cage[octant].z = (((octant >> 2) & 1) * 2) - 1;
            }
            state->objects.push_back(object3d);
            
            // if ((gx == 0) & (gz == 0)) {
            //     for (int octant = 0; octant < 8; octant++) {
            //         int ox = (octant >> 0) & 1;
            //         int oy = (octant >> 1) & 1;
            //         int oz = (octant >> 2) & 1;
            //         if (oy > 0) {
            //             object3d->cage[octant].x *= -0.125f;
            //             object3d->cage[octant].z *= -0.125f;
            //         } else {
            //             object3d->cage[octant].x *= 2.0f;
            //             object3d->cage[octant].z *= 2.0f;
            //         }
            //         object3d->cage[octant].y *= 1.5f;
            //     }
            // }
        }
    }
}

void render_scene_subset(ProgramState* state, struct mat4 proj_matrix, int imin, int imax) {
    struct vec3 cage[8];
    
    if (imin < 0) imin = 0;
    if (imax >= state->objects.size()) imax = state->objects.size() - 1;
    
    for (int index = imin; index <= imax; index++) {
        auto object3d = state->objects[index];
        if (object3d->hide) continue;
        
        auto matrix = trs_matrix(object3d->position, object3d->rotation, object3d->scale);
        psmat4_multiply(&matrix, &proj_matrix, &matrix);
        
        for (int octant = 0; octant < 8; octant++) {
            cage[octant] = transform_vec3(&object3d->cage[octant], &matrix);
        }
        
        DMirEffects effects = object3d->effects;
        if ((effects.max_level < 0) || (effects.max_level > state->max_level)) {
            effects.max_level = state->max_level;
        }
        effects.shape = state->splat_shape;
        
        int group = index;
        dmir_batcher_add(state->dmir_batcher, state->dmir_framebuffer,
            group, (float*)cage, object3d->octree, object3d->root, effects);
    }
    
    dmir_batcher_sort(state->dmir_batcher);
    
    if (state->thread_count == 1) {
        dmir_renderer_draw(state->dmir_renderers[0]);
    } else {
        // This naive approach has a notable overhead
        // (it only becomes faster than single-threaded
        // at 3 threads or above), but it's simple and
        // less prone to segfaults than std::for_each
        // (and this is mainly for demonstration anyway).
        // For less overhead, thread pools would likely
        // be better, but that's not in standard library.
        
        std::vector<std::thread> threads;
        for (int i = 0; i < state->thread_count; i++) {
            threads.emplace_back(dmir_renderer_draw, state->dmir_renderers[i]);
        }
        
        for (auto& thread : threads) {
            thread.join();
        }
    }
}

void calculate_parts(ProgramState* state, int &parts_x, int &parts_y) {
    if (state->thread_case <= 0) {
        parts_x = 1;
        parts_y = 1;
    } else if (state->thread_case <= 1) {
        parts_x = 2;
        parts_y = 1;
    } else if (state->thread_case <= 2) {
        parts_x = 3;
        parts_y = 1;
    } else if (state->thread_case <= 3) {
        parts_x = 2;
        parts_y = 2;
    } else if (state->thread_case <= 4) {
        parts_x = 3;
        parts_y = 2;
    } else if (state->thread_case <= 5) {
        parts_x = 4;
        parts_y = 2;
    } else if (state->thread_case <= 6) {
        parts_x = 3;
        parts_y = 3;
    } else if (state->thread_case <= 7) {
        parts_x = 4;
        parts_y = 3;
    } else {
        parts_x = 4;
        parts_y = 4;
    }
}

void setup_renderers(ProgramState* state) {
    int parts_x = 1;
    int parts_y = 1;
    calculate_parts(state, parts_x, parts_y);
    
    state->thread_count = parts_x * parts_y;
    
    auto framebuffer = state->dmir_framebuffer;
    
    int tiles_x = (framebuffer->size_x + framebuffer->stencil_size_x - 1) / framebuffer->stencil_size_x;
    int tiles_y = (framebuffer->size_y + framebuffer->stencil_size_y - 1) / framebuffer->stencil_size_y;
    
    int index = 0;
    for (int py = 1, last_y = 0; py <= parts_y; py++) {
        int end_y = ((tiles_y * py) / parts_y) * framebuffer->stencil_size_y;
        if (end_y > framebuffer->size_y) end_y = framebuffer->size_y;
        
        for (int px = 1, last_x = 0; px <= parts_x; px++) {
            int end_x = ((tiles_x * px) / parts_x) * framebuffer->stencil_size_x;
            if (end_x > framebuffer->size_x) end_x = framebuffer->size_x;
            
            auto renderer = state->dmir_renderers[index];
            renderer->framebuffer = state->dmir_framebuffer;
            renderer->batcher = state->dmir_batcher;
            renderer->rect = {
                .min_x = last_x,
                .min_y = last_y,
                .max_x = end_x - 1,
                .max_y = end_y - 1,
            };
            
            index++;
            
            last_x = end_x;
        }
        last_y = end_y;
    }
}

void render_scene_to_buffer(ProgramState* state, RGBA32* pixels, int stride) {
    if (state->use_accumulation) {
        auto accum_offset = state->accum_offsets[state->accum_index];
        state->frustum.offset_x = accum_offset.x;
        state->frustum.offset_y = accum_offset.y;
    }
    
    auto proj_matrix = calculate_projection_matrix(state);
    
    setup_renderers(state);
    
    dmir_framebuffer_clear(state->dmir_framebuffer);
    
    dmir_batcher_reset(state->dmir_batcher, state->viewport, state->frustum);
    
    render_scene_subset(state, proj_matrix, 0, state->objects.size()-1);
    
    DMirAffineInfo* affine_infos;
    uint32_t affine_count;
    dmir_batcher_affine_get(state->dmir_batcher, &affine_infos, &affine_count);
    
    int accum_bias = (1 << state->accum_shift) >> 1;
    auto rows_start = std::vector<RGB24*>{};
    rows_start.reserve(state->accum_count);
    auto rows_y = std::vector<RGB24*>{};
    rows_y.reserve(state->accum_count);
    for (int j = 0; j < state->accum_count; j++) {
        int buf_index = (state->accum_index - j + state->accum_count) % state->accum_count;
        auto accum_buf = state->accum_buffers[buf_index];
        rows_start.push_back(accum_buf);
        rows_y.push_back(accum_buf);
    }
    
    int buf_stride = dmir_row_size(state->dmir_framebuffer);
    for (int y = 0; y < state->screen_height; y++) {
        RGBA32* pixels_row = pixels + y * stride;
        
        int buf_row = y * buf_stride;
        DMirDepth* depths_row = state->dmir_framebuffer->depth + buf_row;
        DMirVoxelRef* voxels_row = state->dmir_framebuffer->voxel + buf_row;
        
        for (int j = 0; j < state->accum_count; j++) {
            rows_y[j] = rows_start[j] + (y * state->screen_width);
        }
        auto rows = rows_y.data();
        
        for (int x = 0; x < state->screen_width; x++) {
            RGB24 color;
            if (state->is_depth_mode) {
                #ifdef DMIR_DEPTH_INT32
                color.g = depths_row[x] >> 18;
                #else
                color.g = (int)(8 * 255 * depths_row[x] / state->frustum.max_depth);
                #endif
                color.r = color.g;
                color.b = color.g;
            } else {
                if (voxels_row[x].affine_id < 0) {
                    color = {.r = 0, .g = 196, .b = 255};
                } else {
                    auto affine_info = &affine_infos[voxels_row[x].affine_id];
                    auto octree = affine_info->octree;
                    uint8_t* data_ptr = PTR_INDEX(octree->data, voxels_row[x].address);
                    color = *((RGB24*)data_ptr);
                }
            }
            
            if (state->use_accumulation) {
                int dr = color.r - rows[0][x].r;
                int dg = color.g - rows[0][x].g;
                int db = color.b - rows[0][x].b;
                rows[0][x] = color;
                
                auto weights = ((dr|dg|db) == 0 ? state->accum_weights_lin : state->accum_weights_exp);
                
                dr = accum_bias;
                dg = accum_bias;
                db = accum_bias;
                for (int j = 0; j < state->accum_count; j++) {
                    dr += rows[j][x].r * weights[j];
                    dg += rows[j][x].g * weights[j];
                    db += rows[j][x].b * weights[j];
                }
                
                pixels_row[x].r = dr >> state->accum_shift;
                pixels_row[x].g = dg >> state->accum_shift;
                pixels_row[x].b = db >> state->accum_shift;
                pixels_row[x].a = 255;
            } else {
                pixels_row[x].r = color.r;
                pixels_row[x].g = color.g;
                pixels_row[x].b = color.b;
                pixels_row[x].a = 255;
            }
        }
    }
    
    state->accum_index = (state->accum_index + 1) % state->accum_buffers.size();
}

int64_t render_scene(ProgramState* state) {
    auto time = get_time_ms();
    
    RGBA32* pixels = state->render_buffer.data();
    render_scene_to_buffer(state, pixels, state->screen_width);
    
    #ifdef RGFW_BUFFER
    const int shift = 16;
    int win_w = state->window->r.w;
    int win_h = state->window->r.h;
    if (win_w < 1) win_w = 1;
    if (win_h < 1) win_h = 1;
    if (win_w > RGFW_bufferSize.w) win_w = RGFW_bufferSize.w;
    if (win_h > RGFW_bufferSize.h) win_h = RGFW_bufferSize.h;
    
    int scale_x = ((1 << shift) * state->screen_width) / win_w;
    int offset_x = scale_x >> 1;
    int scale_y = ((1 << shift) * state->screen_height) / win_h;
    int offset_y = scale_y >> 1;
    
    RGBA32* window_pixels = (RGBA32*)state->window->buffer;
    for (int win_y = 0; win_y < win_h; win_y++) {
        int rev_y = (win_h - 1) - win_y;
        int y = (rev_y * scale_y + offset_y) >> shift;
        RGBA32* win_row = window_pixels + win_y * RGFW_bufferSize.w;
        RGBA32* pix_row = pixels + y * state->screen_width;
        for (int win_x = 0; win_x < win_w; win_x++) {
            int x = (win_x * scale_x + offset_x) >> shift;
            win_row[win_x] = pix_row[x];
        }
    }
    #else
    glBindTexture(GL_TEXTURE_2D, state->gl_texture);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, state->screen_width, state->screen_height, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    
    glViewport(0, 0, state->window->r.w, state->window->r.h);
    
    glClear(GL_COLOR_BUFFER_BIT);
    
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, state->gl_texture);
    
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex2f(-1.0f, -1.0f);
    glTexCoord2f(1.0f, 0.0f); glVertex2f( 1.0f, -1.0f);
    glTexCoord2f(1.0f, 1.0f); glVertex2f( 1.0f,  1.0f);
    glTexCoord2f(0.0f, 1.0f); glVertex2f(-1.0f,  1.0f);
    glEnd();
    
    glBindTexture(GL_TEXTURE_2D, 0);
    #endif
    
    time = get_time_ms() - time;
    
    return time;
}

int* make_accum_weights(int count, float base, int denominator) {
    auto fweights = new float[count];
    auto iweights = new int[count];
    
    fweights[0] = base;
    float fsum = fweights[0];
    for (int i = 1; i < count; i++) {
        fweights[i] = fweights[i-1] * base;
        fsum += fweights[i];
    }
    
    int isum = 0;
    for (int i = 0; i < count; i++) {
        iweights[i] = (int)(denominator * fweights[i] / fsum);
        isum += iweights[i];
    }
    
    int difference = denominator - isum;
    for (int i = 0; (i < count) && (difference > 0); i++) {
        iweights[i]++;
        difference--;
    }
    
    delete fweights;
    
    return iweights;
}

void print_stats(ProgramState* state) {
    auto stats = state->dmir_framebuffer->stats;
    std::cout
        << stats[DMIR_NODES_BATCHED] << " batched; "
        << stats[DMIR_NODES_CAGE] << " cages; "
        << stats[DMIR_NODES_ORTHO] << " ortho; "
        << stats[DMIR_OCCLUSIONS_FAILED] << " failed; "
        << stats[DMIR_OCCLUSIONS_PASSED] << " passed; "
        << stats[DMIR_SPLATS_LEAF] << " leaves; "
        << stats[DMIR_SPLATS_1PX] << " 1px; "
        << stats[DMIR_SPLATS_2PX] << " 2px; "
        << stats[DMIR_SPLATS_3PX] << " 3px; "
        << stats[DMIR_SPLATS_4PX] << " 4px; "
        << stats[DMIR_FRAGMENTS_ADDED] << " fragments; "
        << stats[DMIR_FRAGMENTS_WRITTEN] << " writes; "
        << std::endl;
}

void print_state_info(ProgramState* state) {
    std::cout
        << "x: " << state->cam_pos.x << ", "
        << "y: " << state->cam_pos.y << ", "
        << "z: " << state->cam_pos.z << ", "
        << "rx: " << state->cam_rot.x << ", "
        << "ry: " << state->cam_rot.y << ", "
        << "zoom: " << state->cam_zoom << ", "
        << "fov: " << state->cam_zoom_fov << ", "
        << "persp: " << state->frustum.perspective
        << std::endl;
}

void process_event(ProgramState* state) {
    bool cam_updated = false;
    
    auto event = &state->window->event;
    
    if (event->type == RGFW_quit) {
        state->is_running = false;
    } else if (event->type == RGFW_keyPressed) {
        switch (event->key) {
        case RGFW_Escape:
            state->is_running = false;
            break;
        case RGFW_Space:
            state->frustum.perspective = (state->frustum.perspective > 0.5f ? 0 : 1);
            cam_updated = true;
            break;
        case RGFW_Comma:
            state->cam_zoom_fov -= 1;
            cam_updated = true;
            break;
        case RGFW_Period:
            state->cam_zoom_fov += 1;
            cam_updated = true;
            break;
        case RGFW_Bracket:
            state->thread_case -= 1;
            if (state->thread_case < 0) state->thread_case = 0;
            break;
        case RGFW_CloseBracket:
            state->thread_case += 1;
            if (state->thread_case > 8) state->thread_case = 8;
            break;
        case RGFW_Minus:
            state->max_level -= 1;
            if (state->max_level < -1) state->max_level = 16;
            break;
        case RGFW_Equals:
            state->max_level += 1;
            if (state->max_level > 16) state->max_level = -1;
            break;
        case RGFW_Tab:
            state->is_depth_mode = !state->is_depth_mode;
            break;
        case RGFW_F1:
            state->splat_shape = DMIR_SHAPE_POINT;
            break;
        case RGFW_F2:
            state->splat_shape = DMIR_SHAPE_RECT;
            break;
        case RGFW_F3:
            state->splat_shape = DMIR_SHAPE_SQUARE;
            break;
        case RGFW_F4:
            state->splat_shape = DMIR_SHAPE_CIRCLE;
            break;
        case RGFW_F5:
            state->splat_shape = DMIR_SHAPE_CUBE;
            break;
        case RGFW_F12:
            state->use_accumulation = !state->use_accumulation;
            break;
        }
    } else if (event->type == RGFW_mouseButtonPressed) {
        if (event->button == RGFW_mouseLeft) {
            state->is_cam_orbiting = true;
            state->last_mouse_x = event->point.x;
            state->last_mouse_y = event->point.y;
            RGFW_window_mouseHold(state->window, RGFW_AREA(state->window->r.w, state->window->r.h));
            RGFW_window_showMouse(state->window, RGFW_FALSE);
        } else if ((event->button == RGFW_mouseScrollDown) || (event->button == RGFW_mouseScrollUp)) {
            state->cam_zoom += event->scroll;
            cam_updated = true;
        }
    } else if (event->type == RGFW_mouseButtonReleased) {
        if (event->button == RGFW_mouseLeft) {
            state->is_cam_orbiting = false;
            RGFW_window_showMouse(state->window, RGFW_TRUE);
            RGFW_window_mouseUnhold(state->window);
            int restored_x = state->window->r.x + state->last_mouse_x;
            int restored_y = state->window->r.y + state->last_mouse_y;
            RGFW_window_moveMouse(state->window, RGFW_POINT(restored_x, restored_y));
        }
    } else if (event->type == RGFW_mousePosChanged) {
        if (state->is_cam_orbiting) {
            // In "mouse hold" mode, point contains delta rather than absolute position
            state->cam_rot.y += event->point.x * 0.005f;
            state->cam_rot.x += event->point.y * 0.005f;
            cam_updated = true;
        }
    }
    
    if (cam_updated) print_state_info(state);
}

void process_continuous_events(ProgramState* state) {
    auto view_matrix = trs_matrix(state->cam_pos, state->cam_rot, svec3_one());
    auto view_x_axis = get_matrix_vec3(&view_matrix, 0);
    auto view_y_axis = get_matrix_vec3(&view_matrix, 1);
    auto view_z_axis = get_matrix_vec3(&view_matrix, 2);
    float speed = 0.01f;
    
    // A bit of protection against "endless movement" in case
    // some other window suddenly snatches the focus from ours
    if (!state->window->event.inFocus) return;
    
    if (RGFW_isPressed(state->window, RGFW_ShiftL) | RGFW_isPressed(state->window, RGFW_ShiftR)) {
        speed *= 0.1f;
    }
    if (RGFW_isPressed(state->window, RGFW_ControlL) | RGFW_isPressed(state->window, RGFW_ControlR)) {
        speed *= 10;
    }
    
    bool cam_updated = false;
    if (RGFW_isPressed(state->window, RGFW_d)) {
        state->cam_pos = svec3_add(state->cam_pos, svec3_multiply_f(view_x_axis, speed));
        cam_updated = true;
    }
    if (RGFW_isPressed(state->window, RGFW_a)) {
        state->cam_pos = svec3_add(state->cam_pos, svec3_multiply_f(view_x_axis, -speed));
        cam_updated = true;
    }
    if (RGFW_isPressed(state->window, RGFW_r)) {
        state->cam_pos = svec3_add(state->cam_pos, svec3_multiply_f(view_y_axis, speed));
        cam_updated = true;
    }
    if (RGFW_isPressed(state->window, RGFW_f)) {
        state->cam_pos = svec3_add(state->cam_pos, svec3_multiply_f(view_y_axis, -speed));
        cam_updated = true;
    }
    if (RGFW_isPressed(state->window, RGFW_w)) {
        state->cam_pos = svec3_add(state->cam_pos, svec3_multiply_f(view_z_axis, speed));
        cam_updated = true;
    }
    if (RGFW_isPressed(state->window, RGFW_s)) {
        state->cam_pos = svec3_add(state->cam_pos, svec3_multiply_f(view_z_axis, -speed));
        cam_updated = true;
    }
    
    if (cam_updated) print_state_info(state);
}

void process_events(ProgramState* state) {
    while (RGFW_window_checkEvent(state->window)) {
        process_event(state);
    }
    
    process_continuous_events(state);
}

int main(int argc, char* argv[]) {
    ProgramState state = {
        .thread_case = 0,
        .thread_count = 1,
        .screen_width = 640,
        .screen_height = 480,
        .is_running = true,
        .cam_pos = svec3(0, 0, 0),
        .cam_rot = svec3(to_radians(37), to_radians(227), 0),
        .cam_scl = svec3(1, 1, 1),
        .cam_zoom = -4,
        .cam_zoom_fov = 0,
        .max_level = -1,
        .splat_shape = DMIR_SHAPE_RECT,
    };
    
    std::string window_title = "Discrete Mirage";
    
    state.window = RGFW_createWindow(
        window_title.c_str(),
        RGFW_RECT(0, 0, state.screen_width, state.screen_height),
        (u16)(RGFW_CENTER)
    );
    
    state.render_buffer.resize(state.screen_width * state.screen_height, {.r = 0, .g = 0, .b = 0, .a = 255});
    
    #ifndef RGFW_BUFFER
    glGenTextures(1, &state.gl_texture);
    glBindTexture(GL_TEXTURE_2D, state.gl_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, state.screen_width, state.screen_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    #endif
    
    state.viewport = {
        .min_x = 0,
        .min_y = 0,
        .max_x = state.screen_width-1,
        .max_y = state.screen_height-1,
    };
    
    state.frustum = {
        .min_depth = 0.001f,
        .max_depth = 10.0f,
        .focal_extent = 1,
        .focal_depth = 1,
        .perspective = 0,
    };
    
    state.dmir_framebuffer = dmir_framebuffer_make(state.screen_width, state.screen_height);
    state.dmir_batcher = dmir_batcher_make();
    
    for (int i = 0; i < 16; i++) {
        auto dmir_renderer = dmir_renderer_make();
        state.dmir_renderers.push_back(dmir_renderer);
    }
    
    state.accum_count = 4;
    state.accum_index = 0;
    for (int i = 0; i < state.accum_count; i++) {
        auto acc_x = (((i >> 0) & 1) - 0.5f) * 0.5f;
        auto acc_y = (((i >> 1) & 1) - 0.5f) * 0.5f;
        state.accum_offsets.push_back(svec2(acc_x, acc_y));
        auto accum_buf = new RGB24[state.screen_width * state.screen_height];
        state.accum_buffers.push_back(accum_buf);
    }
    
    state.accum_shift = 16;
    state.accum_weights_lin = make_accum_weights(state.accum_count, 1.0f, 1 << state.accum_shift);
    state.accum_weights_exp = make_accum_weights(state.accum_count, 0.65f, 1 << state.accum_shift);
    
    DMirOctree* file_octree = nullptr;
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <file_path>" << std::endl;
    } else {
        file_octree = load_octree(argv[1], octree_load_mode);
    }
    
    create_scene(&state, file_octree);
    
    float max_fps = 30.0f;
    int frame_time_min = (int)(1000.0f / max_fps);
    int next_frame_update = 0;
    
    auto last_title_update = get_time_ms();
    int frame_time_update_period = 500;
    
    int64_t accum_time = 0;
    int64_t accum_count = 0;
    
    state.frame_count = 0;
    
    std::string frame_time_ms = "? ms";
    
    while (state.is_running && !RGFW_window_shouldClose(state.window)) {
        process_events(&state);
        
        accum_time += render_scene(&state);
        accum_count++;
        
        // print_stats(&state);
        
        RGFW_window_swapBuffers(state.window);
        
        auto time = get_time_ms();
        
        if ((time - last_title_update > frame_time_update_period) & (accum_count > 0)) {
            last_title_update = time;
            
            int frame_time = (int)((accum_time / (float)accum_count) + 0.5f);
            accum_time = 0;
            accum_count = 0;
            
            frame_time_ms = std::to_string(frame_time) + " ms";
        }
        
        std::string title = window_title + ": " +
            frame_time_ms + ", " +
            std::to_string(state.thread_count) + " thread(s)";
        RGFW_window_setName(state.window, (char*)(title.c_str()));
        
        auto frame_remainder = next_frame_update - (int)time;
        if (frame_remainder > 0) {
            RGFW_sleep(frame_remainder);
        }
        next_frame_update = time + frame_time_min;
        
        state.frame_count++;
    }
    
    #ifndef RGFW_BUFFER
    glDeleteTextures(1, &state.gl_texture);
    #endif
    
    delete state.accum_weights_lin;
    delete state.accum_weights_exp;
    
    for (int i = 0; i < state.accum_buffers.size(); i++) {
        delete state.accum_buffers[i];
    }
    
    for (int index = 0; index < state.objects.size(); index++) {
        delete state.objects[index];
    }
    
    for (int index = 0; index < state.octrees.size(); index++) {
        delete_octree(state.octrees[index]);
    }
    
    for (int i = 0; i < state.dmir_renderers.size(); i++) {
        auto dmir_renderer = state.dmir_renderers[i];
        dmir_renderer_free(dmir_renderer);
    }
    
    dmir_batcher_free(state.dmir_batcher);
    dmir_framebuffer_free(state.dmir_framebuffer);
    
    RGFW_window_close(state.window);
    
    return 0;
}
