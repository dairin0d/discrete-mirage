// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

#include <stdint.h>
#include <iostream>
#include <fstream>
#include <vector>

#include <SDL2/SDL.h>

extern "C" {
#include <mathc.h>
}

#include <discrete_mirage.h>

#define PTR_OFFSET(array, offset) ((typeof(array))(((char*)(array)) + (offset)))
#define PTR_INDEX(array, index) PTR_OFFSET((array), ((index) * (array##_stride)))

struct mat4 trs_matrix(struct vec3 pos, struct vec3 rot, struct vec3 scl) {
    struct mat4 rotation_x = smat4_identity();
    psmat4_rotation_x(&rotation_x, rot.x);
    struct mat4 rotation_y = smat4_identity();
    psmat4_rotation_y(&rotation_y, rot.y);
    struct mat4 rotation_z = smat4_identity();
    psmat4_rotation_z(&rotation_z, rot.z);
    
    struct mat4 result = smat4_identity();
    psmat4_scale(&result, &result, &scl);
    psmat4_multiply(&result, &rotation_z, &result);
    psmat4_multiply(&result, &rotation_x, &result);
    psmat4_multiply(&result, &rotation_y, &result);
    psmat4_translate(&result, &result, &pos);
    
    return result;
}

struct vec3 get_matrix_vec3(const mfloat_t *m0, int i) {
    i *= 4;
    return svec3(m0[i], m0[i+1], m0[i+2]);
}
struct vec3 get_matrix_vec3(const struct mat4 *m0, int i) {
    return get_matrix_vec3((mfloat_t*)m0, i);
}

struct vec3 transform_vec3(const mfloat_t *v0, const mfloat_t *m0)
{
    mfloat_t x = v0[0];
    mfloat_t y = v0[1];
    mfloat_t z = v0[2];
    return svec3(
        m0[0] * x + m0[4] * y + m0[8] * z + m0[12],
        m0[1] * x + m0[5] * y + m0[9] * z + m0[13],
        m0[2] * x + m0[6] * y + m0[10] * z + m0[14]);
}
struct vec3 transform_vec3(const struct vec3 *v0, const struct mat4 *m0)
{
    return transform_vec3((mfloat_t*)v0, (mfloat_t*)m0);
}

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

uint32_t octree_nodes[] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t octree_masks[] = {255, 255, 255, 255, 255, 255, 255, 255};
uint8_t octree_colors[] = {
    0, 0, 255,
    0, 0, 255,
    0, 0, 255,
    0, 0, 255,
    0, 0, 255,
    0, 0, 255,
    0, 0, 255,
    0, 0, 255,
};

const int OBJECT_COUNT = 5 * 5;
const int CAGE_SIZE = 3*8;
float scene_cages[CAGE_SIZE * OBJECT_COUNT] = {};
float screenspace_cages[CAGE_SIZE * OBJECT_COUNT] = {};
uint32_t scene_roots[OBJECT_COUNT] = {};

struct ProgramState {
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    int buf_w;
    int buf_h;
    float near;
    float far;
    struct vec3 cam_pos;
    struct vec3 cam_rot;
    struct vec3 cam_scl;
    int cam_zoom;
    bool use_perspective;
    bool is_cam_orbiting;
    bool is_running;
    
    DiscreteMirageBuffers buffers;
    DiscreteMirageView view;
    DiscreteMirageOctree octree;
    DiscreteMirageScene scene;
    
    uint8_t* octree_colors;
    uint32_t octree_colors_stride;
    
    bool split_octree_data;
};

static void setup_demo_data(ProgramState* state, int file_size, char* file_data) {
    state->buffers = discrete_mirage_make_buffers(state->buf_w, state->buf_h);
    
    state->view = {
        .xmin = 0,
        .ymin = 0,
        .xmax = state->buf_w-1,
        .ymax = state->buf_h-1,
        .depth_range = state->far - state->near,
        .perspective = 0,
        .max_error = 0,
    };
    
    if ((file_data != nullptr) && (file_size >= 8*(4+4))) {
        state->split_octree_data = true;
        
        int node_count = file_size / 8;
        if (state->split_octree_data) {
            state->octree = {
                .nodes = new uint32_t[node_count],
                .masks = new uint8_t[node_count],
                .nodes_stride = 4,
                .masks_stride = 1,
                .node_count = (uint32_t)node_count,
            };
            state->octree_colors = new uint8_t[node_count*3];
            state->octree_colors_stride = 3;
            for (int node = 0; node < node_count; node++) {
                state->octree.nodes[node] = *((uint32_t*)(file_data+0 + node*8));
                state->octree.masks[node] = *((uint32_t*)(file_data+4 + node*8));
                state->octree_colors[node*3+0] = *((uint32_t*)(file_data+5 + node*8));
                state->octree_colors[node*3+1] = *((uint32_t*)(file_data+6 + node*8));
                state->octree_colors[node*3+2] = *((uint32_t*)(file_data+7 + node*8));
            }
        } else {
            state->octree = {
                .nodes = (uint32_t*)(file_data),
                .masks = (uint8_t*)(file_data + 4),
                .nodes_stride = 8,
                .masks_stride = 8,
                .node_count = (uint32_t)node_count,
            };
            state->octree_colors = (uint8_t*)(file_data + 4+1);
            state->octree_colors_stride = 8;
        }
    } else {
        state->octree = {
            .nodes = octree_nodes,
            .masks = octree_masks,
            .nodes_stride = sizeof(uint32_t),
            .masks_stride = sizeof(uint8_t),
            .node_count = 8,
        };
        state->octree_colors = octree_colors;
        state->octree_colors_stride = 3;
    }
    
    struct vec3* cage_world = (struct vec3*)scene_cages;
    int obj_id = 0;
    const float grid_offset = 1.2f;
    const int grid_extent = 2;
    for (int gz = grid_extent; gz >= -grid_extent; gz--) {
        for (int gx = grid_extent; gx >= -grid_extent; gx--) {
            scene_roots[obj_id] = 0;
            auto obj_cage = cage_world + obj_id * 8;
            for (int octant = 0; octant < 8; octant++) {
                obj_cage[octant].x = (((octant >> 0) & 1) * 2) - 1;
                obj_cage[octant].y = (((octant >> 1) & 1) * 2) - 1;
                obj_cage[octant].z = (((octant >> 2) & 1) * 2) - 1;
                obj_cage[octant].x += gx * grid_offset;
                obj_cage[octant].z += gz * grid_offset;
            }
            obj_id++;
        }
    }
    
    state->scene = {
        .cages = screenspace_cages,
        .roots = scene_roots,
        .cages_stride = (3*8) * sizeof(float),
        .roots_stride = sizeof(uint32_t),
        .object_count = (uint32_t)obj_id,
    };
}

static void project_scene(ProgramState* state) {
    const float zoom_factor = 0.125f;
    float zoom_scale = pow(2.0f, -state->cam_zoom * zoom_factor);
    auto scale_vec = state->cam_scl;
    float cam_distance = 0;
    
    if (state->use_perspective) {
        state->view.perspective = 100;
        scale_vec.x /= state->view.perspective;
        scale_vec.y /= state->view.perspective;
        cam_distance = 2 * zoom_scale;
    } else {
        state->view.perspective = 0;
        scale_vec.x *= zoom_scale;
        scale_vec.y *= zoom_scale;
        cam_distance = state->view.depth_range * 0.5;
    }
    
    auto view_matrix = trs_matrix(state->cam_pos, state->cam_rot, scale_vec);
    auto view_z_axis = get_matrix_vec3(&view_matrix, 2);
    auto offset_axis = svec3_multiply_f(view_z_axis, -cam_distance);
    psmat4_translate(&view_matrix, &view_matrix, &offset_axis);
    
    auto proj_matrix = smat4_inverse(view_matrix);
    
    struct vec3* cage_world = (struct vec3*)scene_cages;
    struct vec3* cage = (struct vec3*)state->scene.cages;
    for (int i = 0; i < (8 * state->scene.object_count); i++) {
        cage[i] = transform_vec3(&cage_world[i], &proj_matrix);
    }
}

static void reinit_renderer(ProgramState* state) {
    if (state->renderer) {
        SDL_DestroyRenderer(state->renderer);
        state->renderer = nullptr;
        state->texture = nullptr;
    }
    
    SDL_Surface* surface = SDL_GetWindowSurface(state->window);
    
    state->renderer = SDL_CreateSoftwareRenderer(surface);
    if (!state->renderer) return;
    
    SDL_RenderSetVSync(state->renderer, 1);
    
    state->texture = SDL_CreateTexture(
        state->renderer,
        SDL_PIXELFORMAT_RGB24,
        SDL_TEXTUREACCESS_STREAMING,
        state->buf_w,
        state->buf_h);
}

static int process_event(void* data, SDL_Event* event)
{
    ProgramState* state = (ProgramState*)data;
    
    if (event->type == SDL_WINDOWEVENT) {
        if (event->window.event == SDL_WINDOWEVENT_RESIZED) {
            SDL_Window* window = SDL_GetWindowFromID(event->window.windowID);
            if (window == state->window) {
                reinit_renderer(state);
            }
        }
    } else if (event->type == SDL_QUIT) {
        state->is_running = false;
    } else if (event->type == SDL_KEYDOWN) {
        switch (event->key.keysym.sym) {
        case SDLK_ESCAPE:
            state->is_running = false;
            break;
        case SDLK_SPACE:
            state->use_perspective = !state->use_perspective;
            break;
        }
    } else if (event->type == SDL_MOUSEBUTTONDOWN) {
        if (event->button.button == SDL_BUTTON_LEFT) {
            state->is_cam_orbiting = true;
        }
    } else if (event->type == SDL_MOUSEBUTTONUP) {
        if (event->button.button == SDL_BUTTON_LEFT) {
            state->is_cam_orbiting = false;
        }
    } else if (event->type == SDL_MOUSEWHEEL) {
        state->cam_zoom += event->wheel.y;
    } else if (event->type == SDL_MOUSEMOTION) {
        if (state->is_cam_orbiting) {
            state->cam_rot.y += event->motion.xrel * 0.005f;
            state->cam_rot.x += event->motion.yrel * 0.005f;
        }
    }
    return 0;
}

static void process_continuous_events(ProgramState* state) {
    const Uint8* currentKeyStates = SDL_GetKeyboardState(nullptr);
    
    auto view_matrix = trs_matrix(state->cam_pos, state->cam_rot, svec3_one());
    auto view_x_axis = get_matrix_vec3(&view_matrix, 0);
    auto view_y_axis = get_matrix_vec3(&view_matrix, 1);
    auto view_z_axis = get_matrix_vec3(&view_matrix, 2);
    float speed = 0.01f;
    if (currentKeyStates[SDL_SCANCODE_LSHIFT] | currentKeyStates[SDL_SCANCODE_RSHIFT]) {
        speed *= 10;
    }
    
    if (currentKeyStates[SDL_SCANCODE_D]) {
        state->cam_pos = svec3_add(state->cam_pos, svec3_multiply_f(view_x_axis, speed));
    } else if(currentKeyStates[SDL_SCANCODE_A]) {
        state->cam_pos = svec3_add(state->cam_pos, svec3_multiply_f(view_x_axis, -speed));
    } else if(currentKeyStates[SDL_SCANCODE_R]) {
        state->cam_pos = svec3_add(state->cam_pos, svec3_multiply_f(view_y_axis, speed));
    } else if(currentKeyStates[SDL_SCANCODE_F]) {
        state->cam_pos = svec3_add(state->cam_pos, svec3_multiply_f(view_y_axis, -speed));
    } else if(currentKeyStates[SDL_SCANCODE_W]) {
        state->cam_pos = svec3_add(state->cam_pos, svec3_multiply_f(view_z_axis, speed));
    } else if(currentKeyStates[SDL_SCANCODE_S]) {
        state->cam_pos = svec3_add(state->cam_pos, svec3_multiply_f(view_z_axis, -speed));
    }
}

static int render_scene(ProgramState* state) {
    void* pixels = nullptr;
    int stride = 0;
    SDL_LockTexture(state->texture, nullptr, &pixels, &stride);
    
    int time = SDL_GetTicks();
    
    project_scene(state);
    discrete_mirage_clear_buffers(state->buffers);
    discrete_mirage_render(state->buffers, state->view, state->octree, state->scene);
    
    int node_count = state->octree.node_count;
    for (int y = 0; y < state->buf_h; y++) {
        uint8_t* pix_row = ((uint8_t*)pixels + y*stride);
        auto buf_row = (state->buf_h - y) * state->buffers.width;
        for (int x = 0; x < state->buf_w; x++) {
            auto pixel = pix_row + x*3;
            auto obj_id = *PTR_INDEX(state->buffers.object, buf_row+x);
            auto node = *PTR_INDEX(state->buffers.node, buf_row+x);
            if (node < node_count) {
                auto rgb = PTR_INDEX(state->octree_colors, node);
                pixel[0] = rgb[0];
                pixel[1] = rgb[1];
                pixel[2] = rgb[2];
            } else {
                pixel[0] = 128;
                pixel[1] = 128;
                pixel[2] = 128;
            }
        }
    }
    
    time = SDL_GetTicks() - time;
    
    SDL_UnlockTexture(state->texture);
    
    return time;
}

int main(int argc, char* argv[]) {
    SDL_LogSetPriority(SDL_LOG_CATEGORY_APPLICATION, SDL_LOG_PRIORITY_INFO);
    
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "SDL_Init fail : %s\n", SDL_GetError());
        return 1;
    }
    
    ProgramState state = {
        .buf_w = 640,
        .buf_h = 480,
        .near = 1,
        .far = 1000,
        .cam_pos = svec3(0, 0, 0),
        .cam_rot = svec3(to_radians(37), to_radians(227), 0),
        .cam_scl = svec3(1, 1, 1),
        .cam_zoom = -4,
        .use_perspective = false,
        .is_running = true,
    };
    
    std::string window_title = "Discrete Mirage";
    
    state.window = SDL_CreateWindow(
        window_title.c_str(),
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        state.buf_w,
        state.buf_h,
        SDL_WINDOW_RESIZABLE
    );
    
    if (!state.window) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Window creation fail : %s\n", SDL_GetError());
        return 1;
    }
    
    reinit_renderer(&state);
    
    if (!state.renderer) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "Render creation for surface fail : %s\n", SDL_GetError());
        return 1;
    }
    
    SDL_AddEventWatch(process_event, &state);
    
    int file_size = 0;
    char* file_data = nullptr;
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <file_path>" << std::endl;
    } else {
        read_file(argv[1], &file_size, &file_data);
    }
    
    setup_demo_data(&state, file_size, file_data);
    
    float max_fps = 30.0f;
    int frame_time_min = (int)(1000.0f / max_fps);
    int next_frame_update = 0;
    
    auto last_title_update = SDL_GetTicks();
    int title_update_period = 1000;
    
    int accum_time = 0;
    int accum_count = 0;
    
    while (state.is_running) {
        SDL_PumpEvents();
        
        if (!state.renderer) break;
        
        process_continuous_events(&state);
        
        SDL_RenderClear(state.renderer);
        
        accum_time += render_scene(&state);
        accum_count++;
        
        // SDL_RenderTexture() in SDL3
        SDL_RenderCopy(state.renderer, state.texture, nullptr, nullptr);
        
        SDL_UpdateWindowSurface(state.window);
        
        auto time = SDL_GetTicks();
        
        if ((time - last_title_update > title_update_period) & (accum_count > 0)) {
            last_title_update = time;
            
            int frame_time = (int)((accum_time / (float)accum_count) + 0.5f);
            accum_time = 0;
            accum_count = 0;
            
            std::string title = window_title + ": " + std::to_string(frame_time) + " ms";
            SDL_SetWindowTitle(state.window, title.c_str());
        }
        
        auto frame_remainder = next_frame_update - (int)time;
        if (frame_remainder > 0) {
            SDL_Delay(frame_remainder);
        }
        next_frame_update = time + frame_time_min;
    }
    
    if (file_data) {
        delete(file_data);
        
        if (state.split_octree_data) {
            delete(state.octree.nodes);
            delete(state.octree.masks);
            delete(state.octree_colors);
        }
    }
    
    discrete_mirage_free_buffers(state.buffers);
    
    SDL_Quit();
    return 0;
}
