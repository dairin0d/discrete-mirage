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
// * F8 - toggle between original and DAG models
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

#include <discrete_mirage.h>

#include "example_implementations.c"

#include "math_utils.cpp"
#include "helper_utils.cpp"

#include "demo_models.cpp"

#ifndef RGFW_BUFFER
UPtr<GLuint> MakeGLTexture() {
    GLuint handle;
    glGenTextures(1, &handle);
    return UPtr<GLuint>(new GLuint(handle), [](GLuint* handle){
        glDeleteTextures(1, handle);
    });
}
#endif

int64_t get_time_ms() {
    return RGFW_getTimeNS() / 1000000;
}

struct Object3D {
    int geometry_id;
    int root;
    DMirEffects effects;
    
    bool hide;
    
    struct vec3 position;
    struct quat rotation;
    struct vec3 scale;
    struct vec3 cage[8];
};

struct ProgramState {
    std::string window_title;
    
    UPtr<RGFW_window> window;
    int last_mouse_x;
    int last_mouse_y;
    
    #ifndef RGFW_BUFFER
    UPtr<GLuint> gl_texture;
    #endif
    
    std::vector<RGBA32> render_buffer;
    
    UPtr<DMirFramebuffer> dmir_framebuffer;
    UPtr<DMirBatcher> dmir_batcher;
    std::vector<UPtr<DMirRenderer>> dmir_renderers;
    
    int thread_case;
    int thread_count;
    int screen_width;
    int screen_height;
    bool is_running;
    
    bool is_test_scene;
    
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
    
    std::vector<DPtr<Object3D>> objects;
    std::vector<UPtr<VoxelModel>> models;
    std::vector<UPtr<VoxelModel>> dag_models;
    bool use_dag;
    
    bool use_accumulation;
    int accum_count;
    int accum_shift;
    int accum_index;
    std::vector<struct vec2> accum_offsets;
    std::vector<std::vector<RGB24>> accum_buffers;
    std::vector<int> accum_weights_lin;
    std::vector<int> accum_weights_exp;
};

struct mat4 calculate_projection_matrix(ProgramState& state) {
    const float zoom_factor = 0.125f;
    float zoom_scale = pow(2.0f, -state.cam_zoom * zoom_factor);
    float zoom_scale_fov = pow(2.0f, -state.cam_zoom_fov * zoom_factor);
    
    float distance_persp = zoom_scale * zoom_scale_fov;
    float distance_ortho = (state.frustum.max_depth - state.frustum.min_depth) * 0.5f;
    state.frustum.focal_depth = lerp(distance_ortho, distance_persp, state.frustum.perspective);
    state.frustum.focal_extent = zoom_scale;
    
    auto view_matrix = trs_matrix(state.cam_pos, state.cam_rot, state.cam_scl);
    auto view_z_axis = get_matrix_vec3(&view_matrix, 2);
    auto offset_axis = svec3_multiply_f(view_z_axis, -state.frustum.focal_depth);
    psmat4_translate(&view_matrix, &view_matrix, &offset_axis);
    
    return smat4_inverse(view_matrix);
}

void load_models(ProgramState& state, std::string model_path) {
    auto model = load_octree(model_path);
    
    if (!model) {
        model = make_procedural_geometry(procedural_sampler, sizeof(ProceduralParameters));
        
        auto proc_geo = (ProceduralGeometry*)model->geometry;
        auto params = (ProceduralParameters*)proc_geo->parameters;
        *params = (ProceduralParameters){
            .animation_time = 0,
            .animation_speed = 0.1f,
            .terrain_detail = 4,
            .terrain_height = 0.25f,
            .sphere_radius = 0.25f,
            .sphere_center = svec3(0.5f, 0.65f, 0.5f),
        };
        
        model->geometry->max_level = 8;
        
        state.max_level = 8;
        state.splat_shape = DMIR_SHAPE_CIRCLE;
    }
    
    state.models.push_back(std::move(model));
}

void recreate_scene(ProgramState& state) {
    state.objects.clear();
    
    int grid_extent = (state.models[0]->geometry_type == MODEL_GEOMETRY_PROCEDURAL ? 0 : 10);
    float grid_offset = (state.is_test_scene ? 1.2f : 2.0f);
    
    DMirEffects effects = {.max_level = -1, .dilation_abs = 0, .dilation_rel = 0};
    int geometry_id = 0;
    for (int gz = grid_extent; gz >= -grid_extent; gz--) {
        for (int gx = grid_extent; gx >= -grid_extent; gx--) {
            auto object3d = new Object3D();
            object3d->geometry_id = geometry_id;
            object3d->root = 0;
            object3d->hide = false;
            object3d->effects = effects;
            object3d->position = svec3(gx * grid_offset, 0, gz * grid_offset);
            object3d->rotation = squat_null();
            object3d->scale = svec3(1, 1, 1);
            for (int octant = 0; octant < 8; octant++) {
                object3d->cage[octant].x = (((octant >> 0) & 1) * 2) - 1;
                object3d->cage[octant].y = (((octant >> 1) & 1) * 2) - 1;
                object3d->cage[octant].z = (((octant >> 2) & 1) * 2) - 1;
            }
            state.objects.push_back(DPtr<Object3D>(object3d));
            
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

void switch_dag_mode(ProgramState& state) {
    state.use_dag = !state.use_dag;
    
    if (state.use_dag && (state.dag_models.size() != state.models.size())) {
        state.dag_models.clear();
        for (auto it = state.models.begin(); it != state.models.end(); ++it) {
            auto model = (*it).get();
            auto dag_model = convert_to_dag(model->geometry, model->data, model->roots[0], true);
            state.dag_models.push_back(std::move(dag_model));
        }
    }
}

void render_scene_subset(ProgramState& state, struct mat4 proj_matrix, int imin, int imax) {
    struct vec3 cage[8];
    
    auto batcher = state.dmir_batcher.get();
    auto framebuffer = state.dmir_framebuffer.get();
    
    if (imin < 0) imin = 0;
    if (imax >= state.objects.size()) imax = state.objects.size() - 1;
    
    auto models = (state.use_dag ? state.dag_models : state.models).data();
    
    for (int index = imin; index <= imax; index++) {
        auto object3d = state.objects[index].get();
        if (object3d->hide) continue;
        
        auto matrix = trs_matrix(object3d->position, object3d->rotation, object3d->scale);
        psmat4_multiply(&matrix, &proj_matrix, &matrix);
        
        for (int octant = 0; octant < 8; octant++) {
            cage[octant] = transform_vec3(&object3d->cage[octant], &matrix);
        }
        
        DMirEffects effects = object3d->effects;
        if ((effects.max_level < 0) || (effects.max_level > state.max_level)) {
            effects.max_level = state.max_level;
        }
        effects.shape = state.splat_shape;
        
        if (object3d->geometry_id >= 0) {
            int group = index;
            auto model = models[object3d->geometry_id].get();
            DMirAddress root = model->roots[object3d->root];
            dmir_batcher_add(batcher, framebuffer,
                group, (float*)cage, model->geometry, root, effects);
        }
    }
    
    dmir_batcher_sort(batcher);
    
    if (state.thread_count == 1) {
        dmir_renderer_draw(state.dmir_renderers[0].get());
    } else {
        // This naive approach has a notable overhead
        // (it only becomes faster than single-threaded
        // at 3 threads or above), but it's simple and
        // less prone to segfaults than std::for_each
        // (and this is mainly for demonstration anyway).
        // For less overhead, thread pools would likely
        // be better, but that's not in standard library.
        
        std::vector<std::thread> threads;
        for (int i = 0; i < state.thread_count; i++) {
            threads.emplace_back(dmir_renderer_draw, state.dmir_renderers[i].get());
        }
        
        for (auto& thread : threads) {
            thread.join();
        }
    }
}

void calculate_parts(ProgramState& state, int &parts_x, int &parts_y) {
    if (state.thread_case <= 0) {
        parts_x = 1;
        parts_y = 1;
    } else if (state.thread_case <= 1) {
        parts_x = 2;
        parts_y = 1;
    } else if (state.thread_case <= 2) {
        parts_x = 3;
        parts_y = 1;
    } else if (state.thread_case <= 3) {
        parts_x = 2;
        parts_y = 2;
    } else if (state.thread_case <= 4) {
        parts_x = 3;
        parts_y = 2;
    } else if (state.thread_case <= 5) {
        parts_x = 4;
        parts_y = 2;
    } else if (state.thread_case <= 6) {
        parts_x = 3;
        parts_y = 3;
    } else if (state.thread_case <= 7) {
        parts_x = 4;
        parts_y = 3;
    } else {
        parts_x = 4;
        parts_y = 4;
    }
}

void setup_renderers(ProgramState& state) {
    int parts_x = 1;
    int parts_y = 1;
    calculate_parts(state, parts_x, parts_y);
    
    state.thread_count = parts_x * parts_y;
    
    auto batcher = state.dmir_batcher.get();
    auto framebuffer = state.dmir_framebuffer.get();
    
    int tiles_x = (framebuffer->size_x + framebuffer->stencil_size_x - 1) / framebuffer->stencil_size_x;
    int tiles_y = (framebuffer->size_y + framebuffer->stencil_size_y - 1) / framebuffer->stencil_size_y;
    
    int index = 0;
    for (int py = 1, last_y = 0; py <= parts_y; py++) {
        int end_y = ((tiles_y * py) / parts_y) * framebuffer->stencil_size_y;
        if (end_y > framebuffer->size_y) end_y = framebuffer->size_y;
        
        for (int px = 1, last_x = 0; px <= parts_x; px++) {
            int end_x = ((tiles_x * px) / parts_x) * framebuffer->stencil_size_x;
            if (end_x > framebuffer->size_x) end_x = framebuffer->size_x;
            
            auto renderer = state.dmir_renderers[index].get();
            renderer->framebuffer = framebuffer;
            renderer->batcher = batcher;
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

void render_scene_to_buffer(ProgramState& state, RGBA32* pixels, int stride) {
    auto batcher = state.dmir_batcher.get();
    auto framebuffer = state.dmir_framebuffer.get();
    
    if (state.use_accumulation) {
        auto accum_offset = state.accum_offsets[state.accum_index];
        state.frustum.offset_x = accum_offset.x;
        state.frustum.offset_y = accum_offset.y;
    }
    
    auto proj_matrix = calculate_projection_matrix(state);
    
    setup_renderers(state);
    
    dmir_framebuffer_clear(framebuffer);
    
    dmir_batcher_reset(batcher, state.viewport, state.frustum);
    
    render_scene_subset(state, proj_matrix, 0, state.objects.size()-1);
    
    DMirAffineInfo* affine_infos;
    uint32_t affine_count;
    dmir_batcher_affine_get(batcher, &affine_infos, &affine_count);
    
    int accum_bias = (1 << state.accum_shift) >> 1;
    auto rows_start = std::vector<RGB24*>{};
    rows_start.reserve(state.accum_count);
    auto rows_y = std::vector<RGB24*>{};
    rows_y.reserve(state.accum_count);
    for (int j = 0; j < state.accum_count; j++) {
        int buf_index = (state.accum_index - j + state.accum_count) % state.accum_count;
        auto accum_buf = state.accum_buffers[buf_index].data();
        rows_start.push_back(accum_buf);
        rows_y.push_back(accum_buf);
    }
    
    auto accum_weights_lin = state.accum_weights_lin.data();
    auto accum_weights_exp = state.accum_weights_exp.data();
    
    auto scene_objects = state.objects.data();
    auto models = (state.use_dag ? state.dag_models : state.models).data();
    
    int buf_stride = dmir_row_size(framebuffer);
    for (int y = 0; y < state.screen_height; y++) {
        RGBA32* pixels_row = pixels + y * stride;
        
        int buf_row = y * buf_stride;
        DMirDepth* depths_row = framebuffer->depth + buf_row;
        DMirVoxelRef* voxels_row = framebuffer->voxel + buf_row;
        
        for (int j = 0; j < state.accum_count; j++) {
            rows_y[j] = rows_start[j] + (y * state.screen_width);
        }
        auto rows = rows_y.data();
        
        for (int x = 0; x < state.screen_width; x++) {
            RGB24 color;
            if (state.is_depth_mode) {
                #ifdef DMIR_DEPTH_INT32
                color.g = depths_row[x] >> 18;
                #else
                color.g = (int)(8 * 255 * depths_row[x] / state.frustum.max_depth);
                #endif
                color.r = color.g;
                color.b = color.g;
            } else {
                if (voxels_row[x].affine_id < 0) {
                    color = {.r = 0, .g = 196, .b = 255};
                } else {
                    auto affine_info = &affine_infos[voxels_row[x].affine_id];
                    auto geometry_id = scene_objects[affine_info->group]->geometry_id;
                    auto voxel_data = models[geometry_id]->data;
                    voxel_data->get(voxel_data, voxels_row[x].voxel_id, &color);
                }
            }
            
            if (state.use_accumulation) {
                int dr = color.r - rows[0][x].r;
                int dg = color.g - rows[0][x].g;
                int db = color.b - rows[0][x].b;
                rows[0][x] = color;
                
                auto weights = ((dr|dg|db) == 0 ? accum_weights_lin : accum_weights_exp);
                
                dr = accum_bias;
                dg = accum_bias;
                db = accum_bias;
                for (int j = 0; j < state.accum_count; j++) {
                    dr += rows[j][x].r * weights[j];
                    dg += rows[j][x].g * weights[j];
                    db += rows[j][x].b * weights[j];
                }
                
                pixels_row[x].r = dr >> state.accum_shift;
                pixels_row[x].g = dg >> state.accum_shift;
                pixels_row[x].b = db >> state.accum_shift;
                pixels_row[x].a = 255;
            } else {
                pixels_row[x].r = color.r;
                pixels_row[x].g = color.g;
                pixels_row[x].b = color.b;
                pixels_row[x].a = 255;
            }
        }
    }
    
    state.accum_index = (state.accum_index + 1) % state.accum_buffers.size();
}

void update_scene(ProgramState& state) {
    auto time = get_time_ms() / 1000.0;
    
    for (auto it = state.models.begin(); it != state.models.end(); ++it) {
        auto model = it->get();
        if (model->geometry_type == MODEL_GEOMETRY_PROCEDURAL) {
            auto proc_geo = (ProceduralGeometry*)model->geometry;
            auto params = (ProceduralParameters*)proc_geo->parameters;
            params->animation_time = time * params->animation_speed;
        }
    }
}

int64_t render_scene(ProgramState& state) {
    update_scene(state);
    
    auto time = get_time_ms();
    
    RGBA32* pixels = state.render_buffer.data();
    render_scene_to_buffer(state, pixels, state.screen_width);
    
    #ifdef RGFW_BUFFER
    const int shift = 16;
    int win_w = state.window->r.w;
    int win_h = state.window->r.h;
    if (win_w < 1) win_w = 1;
    if (win_h < 1) win_h = 1;
    if (win_w > RGFW_bufferSize.w) win_w = RGFW_bufferSize.w;
    if (win_h > RGFW_bufferSize.h) win_h = RGFW_bufferSize.h;
    
    int scale_x = ((1 << shift) * state.screen_width) / win_w;
    int offset_x = scale_x >> 1;
    int scale_y = ((1 << shift) * state.screen_height) / win_h;
    int offset_y = scale_y >> 1;
    
    RGBA32* window_pixels = (RGBA32*)state.window->buffer;
    for (int win_y = 0; win_y < win_h; win_y++) {
        int rev_y = (win_h - 1) - win_y;
        int y = (rev_y * scale_y + offset_y) >> shift;
        RGBA32* win_row = window_pixels + win_y * RGFW_bufferSize.w;
        RGBA32* pix_row = pixels + y * state.screen_width;
        for (int win_x = 0; win_x < win_w; win_x++) {
            int x = (win_x * scale_x + offset_x) >> shift;
            win_row[win_x] = pix_row[x];
        }
    }
    #else
    glBindTexture(GL_TEXTURE_2D, *state.gl_texture.get());
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, state.screen_width, state.screen_height, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    
    glViewport(0, 0, state.window->r.w, state.window->r.h);
    
    glClear(GL_COLOR_BUFFER_BIT);
    
    glEnable(GL_TEXTURE_2D);
    
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

void make_accum_weights(std::vector<int>& out, int count, float base, int denominator) {
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
    
    out.clear();
    out.insert(out.end(), &iweights[0], &iweights[count]);
    
    delete[] fweights;
    delete[] iweights;
}

void print_stats(ProgramState& state) {
    auto stats = state.dmir_framebuffer->stats;
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

void print_state_info(ProgramState& state) {
    std::cout
        << "x: " << state.cam_pos.x << ", "
        << "y: " << state.cam_pos.y << ", "
        << "z: " << state.cam_pos.z << ", "
        << "rx: " << state.cam_rot.x << ", "
        << "ry: " << state.cam_rot.y << ", "
        << "zoom: " << state.cam_zoom << ", "
        << "fov: " << state.cam_zoom_fov << ", "
        << "persp: " << state.frustum.perspective
        << std::endl;
}

void process_event(ProgramState& state) {
    bool cam_updated = false;
    
    auto window = state.window.get();
    auto event = &window->event;
    
    if (event->type == RGFW_quit) {
        state.is_running = false;
    } else if (event->type == RGFW_keyPressed) {
        switch (event->key) {
        case RGFW_Escape:
            state.is_running = false;
            break;
        case RGFW_Space:
            state.frustum.perspective = (state.frustum.perspective > 0.5f ? 0 : 1);
            cam_updated = true;
            break;
        case RGFW_Comma:
            state.cam_zoom_fov -= 1;
            cam_updated = true;
            break;
        case RGFW_Period:
            state.cam_zoom_fov += 1;
            cam_updated = true;
            break;
        case RGFW_Bracket:
            state.thread_case -= 1;
            if (state.thread_case < 0) state.thread_case = 0;
            break;
        case RGFW_CloseBracket:
            state.thread_case += 1;
            if (state.thread_case > 8) state.thread_case = 8;
            break;
        case RGFW_Minus:
            state.max_level -= 1;
            if (state.max_level < -1) state.max_level = 16;
            break;
        case RGFW_Equals:
            state.max_level += 1;
            if (state.max_level > 16) state.max_level = -1;
            break;
        case RGFW_Tab:
            state.is_depth_mode = !state.is_depth_mode;
            break;
        case RGFW_F1:
            state.splat_shape = DMIR_SHAPE_POINT;
            break;
        case RGFW_F2:
            state.splat_shape = DMIR_SHAPE_RECT;
            break;
        case RGFW_F3:
            state.splat_shape = DMIR_SHAPE_SQUARE;
            break;
        case RGFW_F4:
            state.splat_shape = DMIR_SHAPE_CIRCLE;
            break;
        case RGFW_F5:
            state.splat_shape = DMIR_SHAPE_CUBE;
            break;
        case RGFW_F8:
            switch_dag_mode(state);
            break;
        case RGFW_F12:
            state.use_accumulation = !state.use_accumulation;
            break;
        }
    } else if (event->type == RGFW_mouseButtonPressed) {
        if (event->button == RGFW_mouseLeft) {
            state.is_cam_orbiting = true;
            state.last_mouse_x = event->point.x;
            state.last_mouse_y = event->point.y;
            RGFW_window_mouseHold(window, RGFW_AREA(window->r.w, window->r.h));
            RGFW_window_showMouse(window, RGFW_FALSE);
        } else if ((event->button == RGFW_mouseScrollDown) || (event->button == RGFW_mouseScrollUp)) {
            state.cam_zoom += event->scroll;
            cam_updated = true;
        }
    } else if (event->type == RGFW_mouseButtonReleased) {
        if (event->button == RGFW_mouseLeft) {
            state.is_cam_orbiting = false;
            RGFW_window_showMouse(window, RGFW_TRUE);
            RGFW_window_mouseUnhold(window);
            int restored_x = window->r.x + state.last_mouse_x;
            int restored_y = window->r.y + state.last_mouse_y;
            RGFW_window_moveMouse(window, RGFW_POINT(restored_x, restored_y));
        }
    } else if (event->type == RGFW_mousePosChanged) {
        if (state.is_cam_orbiting) {
            // In "mouse hold" mode, point contains delta rather than absolute position
            state.cam_rot.y += event->point.x * 0.005f;
            state.cam_rot.x += event->point.y * 0.005f;
            cam_updated = true;
        }
    }
    
    if (cam_updated) print_state_info(state);
}

void process_continuous_events(ProgramState& state) {
    auto window = state.window.get();
    
    auto view_matrix = trs_matrix(state.cam_pos, state.cam_rot, svec3_one());
    auto view_x_axis = get_matrix_vec3(&view_matrix, 0);
    auto view_y_axis = get_matrix_vec3(&view_matrix, 1);
    auto view_z_axis = get_matrix_vec3(&view_matrix, 2);
    float speed = 0.01f;
    
    // A bit of protection against "endless movement" in case
    // some other window suddenly snatches the focus from ours
    if (!window->event.inFocus) return;
    
    if (RGFW_isPressed(window, RGFW_ShiftL) | RGFW_isPressed(window, RGFW_ShiftR)) {
        speed *= 0.1f;
    }
    if (RGFW_isPressed(window, RGFW_ControlL) | RGFW_isPressed(window, RGFW_ControlR)) {
        speed *= 10;
    }
    
    bool cam_updated = false;
    if (RGFW_isPressed(window, RGFW_d)) {
        state.cam_pos = svec3_add(state.cam_pos, svec3_multiply_f(view_x_axis, speed));
        cam_updated = true;
    }
    if (RGFW_isPressed(window, RGFW_a)) {
        state.cam_pos = svec3_add(state.cam_pos, svec3_multiply_f(view_x_axis, -speed));
        cam_updated = true;
    }
    if (RGFW_isPressed(window, RGFW_r)) {
        state.cam_pos = svec3_add(state.cam_pos, svec3_multiply_f(view_y_axis, speed));
        cam_updated = true;
    }
    if (RGFW_isPressed(window, RGFW_f)) {
        state.cam_pos = svec3_add(state.cam_pos, svec3_multiply_f(view_y_axis, -speed));
        cam_updated = true;
    }
    if (RGFW_isPressed(window, RGFW_w)) {
        state.cam_pos = svec3_add(state.cam_pos, svec3_multiply_f(view_z_axis, speed));
        cam_updated = true;
    }
    if (RGFW_isPressed(window, RGFW_s)) {
        state.cam_pos = svec3_add(state.cam_pos, svec3_multiply_f(view_z_axis, -speed));
        cam_updated = true;
    }
    
    if (cam_updated) print_state_info(state);
}

void process_events(ProgramState& state) {
    auto window = state.window.get();
    
    while (RGFW_window_checkEvent(window)) {
        process_event(state);
    }
    
    process_continuous_events(state);
}

void main_loop(ProgramState& state) {
    auto window = state.window.get();
    
    float max_fps = 30.0f;
    int frame_time_min = (int)(1000.0f / max_fps);
    int next_frame_update = 0;
    
    auto last_title_update = get_time_ms();
    int frame_time_update_period = 500;
    
    int64_t accum_time = 0;
    int64_t accum_count = 0;
    
    state.frame_count = 0;
    
    std::string frame_time_ms = "? ms";
    
    while (state.is_running && !RGFW_window_shouldClose(window)) {
        process_events(state);
        
        recreate_scene(state);
        
        accum_time += render_scene(state);
        accum_count++;
        
        // print_stats(&state);
        
        RGFW_window_swapBuffers(window);
        
        auto time = get_time_ms();
        
        if ((time - last_title_update > frame_time_update_period) & (accum_count > 0)) {
            last_title_update = time;
            
            int frame_time = (int)((accum_time / (float)accum_count) + 0.5f);
            accum_time = 0;
            accum_count = 0;
            
            frame_time_ms = std::to_string(frame_time) + " ms";
        }
        
        std::string model_type = "OCT";
        if (state.use_dag) {
            model_type = "DAG";
        } else if (state.models[0]->geometry_type == MODEL_GEOMETRY_PROCEDURAL) {
            model_type = "PRC";
        }
        
        std::string title = state.window_title + ": " +
            model_type + " " +
            frame_time_ms + ", " +
            std::to_string(state.thread_count) + " thread(s)";
        RGFW_window_setName(window, (char*)(title.c_str()));
        
        auto frame_remainder = next_frame_update - (int)time;
        if (frame_remainder > 0) {
            RGFW_sleep(frame_remainder);
        }
        next_frame_update = time + frame_time_min;
        
        state.frame_count++;
    }
}

int main(int argc, char* argv[]) {
    ProgramState state = {
        .window_title = "Discrete Mirage",
        .window = UPtr<RGFW_window>(nullptr, nullptr),
        #ifndef RGFW_BUFFER
        .gl_texture = UPtr<GLuint>(nullptr, nullptr),
        #endif
        .dmir_framebuffer = UPtr<DMirFramebuffer>(nullptr, nullptr),
        .dmir_batcher = UPtr<DMirBatcher>(nullptr, nullptr),
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
    
    state.window = UPtr<RGFW_window>(
        RGFW_createWindow(
            state.window_title.c_str(),
            RGFW_RECT(0, 0, state.screen_width, state.screen_height),
            (u16)(RGFW_CENTER)
        ), RGFW_window_close);
    
    state.render_buffer.resize(state.screen_width * state.screen_height, {.r = 0, .g = 0, .b = 0, .a = 255});
    
    #ifndef RGFW_BUFFER
    state.gl_texture = MakeGLTexture();
    glBindTexture(GL_TEXTURE_2D, *state.gl_texture.get());
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
        .max_depth = 100.0f,
        .focal_extent = 1,
        .focal_depth = 1,
        .perspective = 0,
    };
    
    dmir_lookups_initialize();
    
    state.dmir_framebuffer = UPtr<DMirFramebuffer>(
        dmir_framebuffer_make(state.screen_width, state.screen_height),
        &dmir_framebuffer_free);
    
    state.dmir_batcher = UPtr<DMirBatcher>(
        dmir_batcher_make(),
        &dmir_batcher_free);
    
    for (int i = 0; i < 16; i++) {
        auto renderer = UPtr<DMirRenderer>(
            dmir_renderer_make(),
            &dmir_renderer_free);
        state.dmir_renderers.push_back(std::move(renderer));
    }
    
    state.accum_count = 4;
    state.accum_index = 0;
    state.accum_buffers.resize(state.accum_count);
    for (int i = 0; i < state.accum_count; i++) {
        auto acc_x = (((i >> 0) & 1) - 0.5f) * 0.5f;
        auto acc_y = (((i >> 1) & 1) - 0.5f) * 0.5f;
        state.accum_offsets.push_back(svec2(acc_x, acc_y));
        state.accum_buffers[i].resize(state.screen_width * state.screen_height);
    }
    
    state.accum_shift = 16;
    make_accum_weights(state.accum_weights_lin, state.accum_count, 1.0f, 1 << state.accum_shift);
    make_accum_weights(state.accum_weights_exp, state.accum_count, 0.65f, 1 << state.accum_shift);
    
    if (argc <= 1) {
        std::cerr << "Usage: " << argv[0] << " <file_path>" << std::endl;
    }
    
    std::string model_path = (argc > 1 ? argv[1] : "");
    
    state.is_test_scene = (argc > 2) && (strcmp(argv[2], "TEST_SCENE") == 0);
    
    load_models(state, model_path);
    
    main_loop(state);
    
    return 0;
}
