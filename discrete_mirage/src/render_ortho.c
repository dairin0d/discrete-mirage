// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

// Specializing based on the use of local stencil or cube mode
// (basically, anything that switches to a different function
// midway) does not seem to improve performance (might even be
// slightly worse?)

static void RENDER_ORTHO(MODE)(LocalVariables* v, OrthoStackItem* stack,
    Stencil* local_stencil, Bool is_initialized, Bool is_local)
{
    OrthoStackItem* stack_start = stack;
    
    if (is_initialized) {
        stack_start--;
    } else {
        v->octant = 0;
        goto skip_initialization;
    }
    
    do {
        if (stack->queue == 0) {
            stack--;
            continue;
        }
        v->octant = stack->queue & 7;
        stack->queue >>= 4;
        v->position.x = stack->center.x + stack->deltas[v->octant].x;
        v->position.y = stack->center.y + stack->deltas[v->octant].y;
        v->position.z = stack->center.z + stack->deltas[v->octant].z;
        #if defined(MODE_TREE)
        v->node_ref = stack->traversal.node[v->octant];
        #elif defined(MODE_EVAL)
        node_box_child(&stack->node_box, v->octant, &v->node_box);
        #endif
        
        skip_initialization:;
        
        STAT_INCREMENT(v->framebuffer, DMIR_NODES_ORTHO);
        
        if ((v->position.z >= stack->max_z) | (v->position.z <= stack->min_z)) continue;
        
        // Calculate screen-space bounds (in pixels)
        RECT_FROM_POINT(v->rect, v->position, stack->extent.x, stack->extent.y);
        
        // Calculate node size (in pixels)
        SInt size_x = v->rect.max_x - v->rect.min_x;
        SInt size_y = v->rect.max_y - v->rect.min_y;
        v->size_max = MAX(size_x, size_y);
        
        #ifdef STENCIL_LOCAL_SIZE
        if (is_local)
        {
            // if (v->rect.min_x > v->rect.max_x) continue;
            
            // Empirically, this seems to offer no performance benefit
            // #ifdef DMIR_SKIP_OCCLUDED_ROWS
            // MAX_UPDATE(v->rect.min_y, stack->rect.min_y);
            // #endif
            
            v->row_mask = (STENCIL_CLEAR << v->rect.min_x) ^ ((STENCIL_CLEAR << 1) << v->rect.max_x);
            // if (v->row_mask == 0) continue;
            
            for (; v->rect.min_y <= v->rect.max_y; v->rect.min_y++) {
                if (v->local_stencil[v->rect.min_y] & v->row_mask) goto not_occluded;
            }
            STAT_INCREMENT(v->framebuffer, DMIR_OCCLUSIONS_FAILED);
            continue;
            not_occluded:;
            STAT_INCREMENT(v->framebuffer, DMIR_OCCLUSIONS_PASSED);
        }
        else
        #endif
        {
            // Clamp to viewport
            RECT_CLIP(v->rect, stack->rect);
            
            // Skip if not visible
            if ((v->rect.min_x > v->rect.max_x) | (v->rect.min_y > v->rect.max_y)) continue;
            
            if (is_occluded_quad(v->framebuffer, &v->rect, v->position.z - stack->extent.z)) continue;
        }
        
        v->is_cube = stack->is_cube;
        
        SInt is_splat = (v->size_max == 0) | (stack->level == v->max_stack_level);
        SInt is_leaf = FALSE;
        
        if (v->is_cube) {
            v->mask = 255;
        } else {
            is_leaf |= (stack->level == v->max_level);
            
            #if defined(MODE_EVAL)
            int32_t eval_result = v->geometry->evaluate(v->geometry, &v->node_box, &stack->traversal.data[v->octant]);
            if (eval_result < 0) continue;
            is_leaf |= (eval_result == 0);
            v->mask = 255;
            #else
            v->mask = stack->traversal.mask[v->octant];
            is_leaf |= !v->mask;
            #endif
            
            if (is_leaf & (v->shape == DMIR_SHAPE_CUBE)) {
                v->mask = 255;
                v->is_cube = TRUE;
                is_leaf = FALSE;
            }
        }
        
        // Whatever happens, don't traverse past max stack level
        if (is_leaf | is_splat) {
            v->data_ref = stack->traversal.data[v->octant];
            draw_splat(v, stack);
            continue;
        }
        
        #if defined(USE_MAP) && !defined(MODE_EVAL)
        if (v->size_max < v->map.size8) {
            v->data_ref = stack->traversal.data[v->octant];
            draw_map8(v, stack);
            continue;
        }
        if (v->size_max < v->map.size64) {
            v->data_ref = stack->traversal.data[v->octant];
            draw_map64(v, stack);
            continue;
        }
        #endif
        
        stack++;
        stack->rect = v->rect;
        stack->node_box = v->node_box;
        stack->center = v->position;
        stack->is_cube = v->is_cube;
        
        #if defined(MODE_TREE)
        if (v->is_cube) {
            v->data_ref = (stack-1)->traversal.data[v->octant];
            for (SInt i = 0; i < 8; i++) {
                stack->traversal.mask[i] = 255;
                stack->traversal.node[i] = v->node_ref;
                stack->traversal.data[i] = v->data_ref;
            }
        } else if (!v->geometry->traverse_next(v->geometry, &(stack-1)->traversal, v->octant, &stack->traversal)) {
            stack--;
            continue;
        }
        #elif defined(MODE_EVAL)
        v->data_ref = (stack-1)->traversal.data[v->octant];
        for (SInt i = 0; i < 8; i++) {
            stack->traversal.mask[i] = 255;
            stack->traversal.node[i] = v->node_ref;
            stack->traversal.data[i] = v->data_ref;
        }
        #endif
        
        stack->queue = v->queues_forward[v->mask].octants;
        
        #ifdef STENCIL_LOCAL_SIZE
        if ((!is_local) && (v->size_max < (STENCIL_LOCAL_SIZE - 2))) {
            v->local_stencil = local_stencil;
            initialize_ortho_local(v, stack);
            
            RENDER_ORTHO(MODE)(v, stack, local_stencil, TRUE, TRUE);
            
            v->local_stencil = NULL;
            v->offset_x = 0;
            v->offset_y = 0;
            v->clip_rect = stack_start->rect;
            
            stack--;
        }
        #endif
    } while (stack > stack_start);
}
