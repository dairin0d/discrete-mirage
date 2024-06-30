// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

#include <string>

#include <SDL2/SDL.h>

struct ProgramState {
    SDL_Window* window;
    SDL_Renderer* renderer;
    SDL_Texture* texture;
    int screen_width;
    int screen_height;
    bool is_running;
};

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
        state->screen_width,
        state->screen_height);
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
        }
    }
    return 0;
}

static int render_scene(ProgramState* state) {
    void* pixels = nullptr;
    int stride = 0;
    SDL_LockTexture(state->texture, nullptr, &pixels, &stride);
    
    int time = SDL_GetTicks();
    
    for (int y = 0; y < state->screen_height; y++) {
        uint8_t* pix_row = ((uint8_t*)pixels + y*stride);
        for (int x = 0; x < state->screen_width; x++) {
            auto pixel = pix_row + x*3;
            pixel[0] = 128;
            pixel[1] = 128;
            pixel[2] = 128;
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
