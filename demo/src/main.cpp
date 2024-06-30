// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

#include <string>
#include <stdint.h>

#include <SDL2/SDL.h>

#include "math_utils.hpp"

struct ProgramState {
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    int screen_width;
    int screen_height;
    bool is_running;
    
    float near;
    float far;
    struct vec3 cam_pos;
    struct vec3 cam_rot;
    struct vec3 cam_scl;
    int cam_zoom;
    bool use_perspective;
    float perspective_const;
    float perspective_scale;
    bool is_cam_orbiting;
};

struct mat4 project_scene(ProgramState* state) {
    const float zoom_factor = 0.125f;
    float zoom_scale = pow(2.0f, -state->cam_zoom * zoom_factor);
    auto scale_vec = state->cam_scl;
    float cam_distance = 0;
    
    if (state->use_perspective) {
        state->perspective_const = 1;
        state->perspective_scale = 100;
        scale_vec.x /= state->perspective_scale;
        scale_vec.y /= state->perspective_scale;
        cam_distance = 2 * zoom_scale;
    } else {
        state->perspective_const = 1;
        state->perspective_scale = 0;
        scale_vec.x *= zoom_scale;
        scale_vec.y *= zoom_scale;
        cam_distance = (state->far - state->near) * 0.5;
    }
    
    auto view_matrix = trs_matrix(state->cam_pos, state->cam_rot, scale_vec);
    auto view_z_axis = get_matrix_vec3(&view_matrix, 2);
    auto offset_axis = svec3_multiply_f(view_z_axis, -cam_distance);
    psmat4_translate(&view_matrix, &view_matrix, &offset_axis);
    
    auto proj_matrix = smat4_inverse(view_matrix);
    
    return proj_matrix;
}

void reinit_renderer(ProgramState* state) {
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
        state->screen_width,
        state->screen_height);
}

int process_event(void* data, SDL_Event* event)
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

void process_continuous_events(ProgramState* state) {
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

int render_scene(ProgramState* state) {
    void* pixels = nullptr;
    int stride = 0;
    SDL_LockTexture(state->texture, nullptr, &pixels, &stride);
    
    int time = SDL_GetTicks();
    
    auto proj_matrix = project_scene(state);
    
    for (int y = 0; y < state->screen_height; y++) {
        uint8_t* pix_row = ((uint8_t*)pixels + y*stride);
        for (int x = 0; x < state->screen_width; x++) {
            auto pixel = pix_row + x*3;
            pixel[0] = 128;
            pixel[1] = 128;
            pixel[2] = 128;
        }
    }
    
    float half_x = state->screen_width * 0.5f;
    float half_y = state->screen_height * 0.5f;
    float xy_scale = (half_x > half_y) ? half_x : half_y;
    
    const float grid_offset = 1.2f;
    const int grid_extent = 2;
    for (int gz = grid_extent; gz >= -grid_extent; gz--) {
        for (int gx = grid_extent; gx >= -grid_extent; gx--) {
            struct vec3 vertex;
            int xmin = INT32_MAX, ymin = INT32_MAX, xmax = INT32_MIN, ymax = INT32_MIN;
            float zmin = INFINITY;
            
            for (int octant = 0; octant < 8; octant++) {
                vertex.x = (((octant >> 0) & 1) * 2) - 1;
                vertex.y = (((octant >> 1) & 1) * 2) - 1;
                vertex.z = (((octant >> 2) & 1) * 2) - 1;
                vertex.x += gx * grid_offset;
                vertex.z += gz * grid_offset;
                vertex = transform_vec3(&vertex, &proj_matrix);
                
                float scale = xy_scale / (state->perspective_const + state->perspective_scale * vertex.z);
                int x = (int)(half_x + vertex.x * scale);
                int y = (int)(half_y + vertex.y * scale);
                xmin = (x < xmin ? x : xmin);
                ymin = (y < ymin ? y : ymin);
                xmax = (x > xmax ? x : xmax);
                ymax = (y > ymax ? y : ymax);
                
                zmin = (vertex.z < zmin ? vertex.z : zmin);
            }
            
            if (zmin < 0) continue;
            
            xmin = (xmin < 0 ? 0 : xmin);
            ymin = (ymin < 0 ? 0 : ymin);
            xmax = (xmax >= state->screen_width ? state->screen_width - 1 : xmax);
            ymax = (ymax >= state->screen_height ? state->screen_height - 1 : ymax);
            
            for (int y = ymin; y <= ymax; y++) {
                uint8_t* pix_row = ((uint8_t*)pixels + y*stride);
                for (int x = xmin; x <= xmax; x++) {
                    auto pixel = pix_row + x*3;
                    pixel[0] = 0;
                    pixel[1] = 255;
                    pixel[2] = 0;
                }
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
        .screen_width = 640,
        .screen_height = 480,
        .is_running = true,
        .near = 1,
        .far = 1000,
        .cam_pos = svec3(0, 0, 0),
        .cam_rot = svec3(to_radians(37), to_radians(227), 0),
        .cam_scl = svec3(1, 1, 1),
        .cam_zoom = -4,
        .use_perspective = false,
    };
    
    std::string window_title = "Discrete Mirage";
    
    state.window = SDL_CreateWindow(
        window_title.c_str(),
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        state.screen_width,
        state.screen_height,
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
    
    float max_fps = 60.0f;
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
    
    SDL_Quit();
    return 0;
}
