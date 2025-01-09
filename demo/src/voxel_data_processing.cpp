// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

#pragma once

#include <stdint.h>
#include <algorithm>
#include <vector>
#include <deque>

#include <discrete_mirage.h>

#include "example_implementations.c"

namespace dmir {

// If root < octree.count, extracts data of that specific subtree
// in depth-first traversal order. Otherwise, extracts all data in the
// original order (to stay in sync with the corresponding octree nodes).
void octree_extract_voxel_data(DMirOctree* octree, DMirAddress root,
    DMirVoxelData* voxel_data, uint32_t offset, int32_t size,
    std::deque<uint8_t>& out)
{
    uint32_t item_size = voxel_data->item_size;
    if (size < 0) size = item_size;
    
    std::vector<uint8_t> data_item;
    data_item.resize(std::max(item_size, offset+size));
    auto data_item_ptr = data_item.data();
    auto subdata = data_item_ptr + offset;
    
    if (root < octree->count) {
        DMirAddress address_stack[8*(DMIR_LEVEL_MAX + 1)];
        DMirAddress* stack = address_stack;
        stack[0] = root;
        
        do {
            DMirAddress address = *stack;
            stack--;
            
            uint8_t mask = *PTR_INDEX(octree->mask, address);
            
            voxel_data->get(&voxel_data, address, data_item_ptr);
            out.insert(out.end(), subdata, subdata + size);
            
            if (mask == 0) continue;
            
            address = *PTR_INDEX(octree->addr, address);
            
            // For correct order, nodes on stack must be added in reverse
            if (octree->is_packed) {
                int count = 0;
                for (int i = 0; i < 8; i++) {
                    if ((mask & (1 << i)) == 0) continue;
                    stack++;
                    stack[0] = address + count;
                    count++;
                }
                for (int i_min = -(count-1), i_max = 0; i_min < i_max; i_min++, i_max--) {
                    auto temp = stack[i_min];
                    stack[i_min] = stack[i_max];
                    stack[i_max] = temp;
                }
            } else {
                for (int i = 7; i >= 0; i--) {
                    if ((mask & (1 << i)) == 0) continue;
                    stack++;
                    stack[0] = address + i;
                }
            }
        } while (stack >= address_stack);
    } else {
        for (DMirAddress address = 0; address < octree->count; address++) {
            voxel_data->get(&voxel_data, address, data_item_ptr);
            out.insert(out.end(), subdata, subdata + size);
        }
    }
}

void extract_voxel_data(DMirGeometry* geometry, DMirAddress root,
    DMirVoxelData* voxel_data, uint32_t offset, int32_t size,
    std::deque<uint8_t>& out)
{
    uint32_t item_size = voxel_data->item_size;
    if (size < 0) size = item_size;
    
    std::vector<uint8_t> data_item;
    data_item.resize(std::max(item_size, offset+size));
    
    struct VisitorState {
        DMirVoxelData* voxel_data;
        uint32_t offset;
        int32_t size;
        uint8_t* data_item;
        std::deque<uint8_t>* out;
    };
    
    VisitorState state = {
        .voxel_data = voxel_data,
        .offset = offset,
        .size = size,
        .data_item = data_item.data(),
        .out = &out,
    };
    
    DMirVisitor visitor = {.level_limit = -1, .state = &state};
    visitor.visit = [](DMirVisitor* visitor, DMirVisitorNodeInfo* node_info) {
        auto state = (VisitorState*)visitor->state;
        state->voxel_data->get(state->voxel_data, node_info->data_ref, state->data_item);
        auto subdata = state->data_item + state->offset;
        state->out->insert(state->out->begin(), subdata, subdata + state->size);
        return (DMirBool)true;
    };
    
    dmir_visit(geometry, root, &visitor, true);
}

} // namespace dmir
