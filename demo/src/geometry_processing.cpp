// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

#pragma once

#include <stdint.h>
#include <algorithm>
#include <vector>
#include <unordered_set>
#include <memory>

#include <iostream>

#include <discrete_mirage.h>

#include "example_implementations.c"

namespace dmir {

const uint32_t DMIR_LEVEL_MASK = ((1 << DMIR_LEVEL_BITS) - 1);
const uint32_t DMIR_RARITY_MASK = (UINT32_MAX >> DMIR_LEVEL_BITS);

class DAGBuilder {
public:
    DAGBuilder();
    ~DAGBuilder();
    
    DAGBuilder(const DAGBuilder&) = delete;
    DAGBuilder& operator=(const DAGBuilder&) = delete;
    DAGBuilder(DAGBuilder&& other) noexcept = default;
    DAGBuilder& operator=(DAGBuilder&&) noexcept = default;
    
    void add(const DMirGeometry* geometry, uint32_t address);
    void build(bool isCompact, DMirVoxelDAG& dag, std::vector<DMirAddress>& roots);
    void report();
    void clear();

private:
    // NodeRef represents a position in the buffer where a node starts
    using NodeRef = uint32_t;
    
    // Custom hash functor for NodeRef
    struct NodeRefHash {
        const std::vector<uint32_t>* m_buffer;
        
        explicit NodeRefHash(const std::vector<uint32_t>& buffer) 
            : m_buffer(&buffer) {}
        
        size_t operator()(const NodeRef& ref) const {
            uint32_t dagLevel = (*m_buffer)[ref] >> (32 - DMIR_LEVEL_BITS);
            uint32_t childCount = ((*m_buffer)[ref+1] >> 8) & 15;
            uint32_t recordSize = (dagLevel ? 3 + childCount : 2 + (childCount+3)/4);
            
            size_t hash = 17;
            for (size_t i = 1; i < recordSize; ++i) {
                hash = hash * 31 + (*m_buffer)[ref + i];
            }
            
            return hash;
        }
    };
    
    // Custom equality functor for NodeRef
    struct NodeRefEqual {
        const std::vector<uint32_t>* m_buffer;
        
        explicit NodeRefEqual(const std::vector<uint32_t>& buffer) 
            : m_buffer(&buffer) {}
        
        bool operator()(const NodeRef& refA, const NodeRef& refB) const {
            uint32_t dagLevelA = (*m_buffer)[refA] >> (32 - DMIR_LEVEL_BITS);
            uint32_t dagLevelB = (*m_buffer)[refB] >> (32 - DMIR_LEVEL_BITS);
            if (dagLevelA != dagLevelB) return false;
            
            uint32_t childCountA = ((*m_buffer)[refA+1] >> 8) & 15;
            uint32_t childCountB = ((*m_buffer)[refB+1] >> 8) & 15;
            if (childCountA != childCountB) return false;
            
            uint32_t recordSize = (dagLevelA ? 3 + childCountA : 2 + (childCountA+3)/4);
            
            for (size_t i = 1; i < recordSize; ++i) {
                if ((*m_buffer)[refA + i] != (*m_buffer)[refB + i]) return false;
            }
            
            return true;
        }
    };
    
    uint64_t m_nodeCount;
    uint64_t m_mergeCount;
    uint64_t m_totalLeafNodes;
    uint64_t m_totalLeafSizes;
    uint64_t m_voxelCount;
    uint64_t m_sizeInBytes;
    
    std::vector<uint8_t> m_maskFlips;
    
    // References to the root nodes (level, index, flip)
    std::vector<NodeRef> m_roots;
    
    // Buffer storing all node data contiguously
    std::vector<uint32_t> m_buffer;
    
    // The set of known/unique nodes
    std::unordered_set<NodeRef, NodeRefHash, NodeRefEqual> m_knownNodes;
    
    // Helper methods
    uint8_t flipMask(uint8_t mask, uint32_t flip);
    uint32_t findCanonicalOrder(uint32_t* children,  bool as_masks);
    void reduceRarity(NodeRef ref);
    static DMirBool visit(DMirVisitor* visitor, DMirVisitorNodeInfo* node_info);
};

DAGBuilder::DAGBuilder()
    : m_knownNodes(1024, NodeRefHash(m_buffer), NodeRefEqual(m_buffer))
{
    m_maskFlips.reserve(8 * 256);
    for (uint32_t flip = 0; flip < 8; flip++) {
        for (uint32_t maskBase = 0; maskBase < 256; maskBase++) {
            uint8_t mask = maskBase;
            if (flip & 0b001) mask = ((mask & 0b10101010) >> 1) | ((mask & 0b01010101) << 1);
            if (flip & 0b010) mask = ((mask & 0b11001100) >> 2) | ((mask & 0b00110011) << 2);
            if (flip & 0b100) mask = ((mask & 0b11110000) >> 4) | ((mask & 0b00001111) << 4);
            m_maskFlips[(flip << 8) | maskBase] = mask;
        }
    }
    
    m_buffer.reserve(1024 * 1024);
    
    clear();
}

DAGBuilder::~DAGBuilder() {
    clear();
}

void DAGBuilder::clear() {
    m_nodeCount = 0;
    m_mergeCount = 0;
    m_totalLeafNodes = 0;
    m_totalLeafSizes = 0;
    m_voxelCount = 0;
    m_roots.clear();
    m_buffer.clear();
    m_knownNodes.clear();
}

inline uint8_t DAGBuilder::flipMask(uint8_t mask, uint32_t flip) {
    return m_maskFlips[(flip << 8) | mask];
}

uint32_t DAGBuilder::findCanonicalOrder(uint32_t* children, bool as_masks) {
    // Possible flips, in the order of preferred priority
    uint32_t flips = 0b0'111'110'101'100'011'010'001'000;
    uint32_t flipsSize = 8*3;
    
    // The sequence with the smallest lexicographical order is "canonical"
    for (uint32_t lexIndex = 0; lexIndex < 8; lexIndex++) {
        uint32_t minLexValue = 0;
        
        uint32_t flipShift = 0;
        for (; flipShift < flipsSize; flipShift += 3) {
            uint32_t flip = (flips >> flipShift) & 7;
            
            // Here, the children array stores their mirror bits in
            // bits 0..2, and the actual node reference in bits 3..31
            uint32_t lexValue = children[lexIndex ^ flip];
            if (lexValue < UINT32_MAX) {
                lexValue = (as_masks ? flipMask(lexValue, flip) : lexValue ^ flip);
            }
            
            if (flipShift == 0) {
                // First flip item, just initialize the min value
                minLexValue = lexValue;
            } else if (lexValue < minLexValue) {
                // Encountered a smaller value; discard all preceding candidates
                minLexValue = lexValue;
                flips >>= flipShift;
                flipsSize -= flipShift;
                flipShift = 0;
            } else if (lexValue > minLexValue) {
                // Encountered a bigger value; eliminate this candidate
                flips = ((flips >> (flipShift+3)) << flipShift) | (flips & ~(UINT32_MAX << flipShift));
                flipsSize -= 3;
                flipShift -= 3; // compensate for the loop's increment
            }
        }
        
        if (flipsSize == 3) break; // only one candidate remains
    }
    
    return flips & 7;
}

void DAGBuilder::reduceRarity(NodeRef ref) {
    uint32_t sortKey = m_buffer[ref];
    uint32_t rarity = sortKey & DMIR_RARITY_MASK;
    m_buffer[ref] = (sortKey & ~DMIR_RARITY_MASK) | (rarity > 0 ? rarity - 1 : 0);
}

DMirBool DAGBuilder::visit(DMirVisitor* visitor, DMirVisitorNodeInfo* node_info) {
    DAGBuilder* self = (DAGBuilder*)visitor->state;
    
    self->m_voxelCount++;
    
    int32_t level = visitor->level_limit - node_info->node_box.level;
    if ((level < 2) & (level != visitor->level_limit)) {
        return true;
    }
    
    // 2nd level (grandparent of leaves) is the minimal addressable level in our DAG
    uint32_t dagLevel = std::max(level - 2, 0);
    uint32_t levelTag = dagLevel << (32 - DMIR_LEVEL_BITS);
    uint32_t sortKey = levelTag | (UINT32_MAX >> DMIR_LEVEL_BITS);
    
    uint32_t info[11];
    uint32_t recordSize = 0;
    uint32_t flip = 0;
    
    uint32_t count = 0;
    uint32_t children[8];
    
    if (level <= 2) {
        for (int i = 0; i < 8; i++) {
            if (node_info->mask & (1 << i)) {
                children[i] = (node_info->subnode_masks >> (i*8)) & 255;
            } else {
                children[i] = UINT32_MAX;
            }
        }
        
        flip = self->findCanonicalOrder(children, true);
        
        uint8_t mask = self->flipMask(node_info->mask, flip);
        
        uint8_t* info_children = (uint8_t*)(info + 2);
        
        // Clear any potential leftovers
        *((uint64_t*)info_children) = 0;
        
        count = 0;
        for (int i = 0; i < 8; i++) {
            if ((mask & (1 << i)) == 0) continue;
            info_children[count] = self->flipMask(children[i ^ flip], flip);
            count++;
        }
        
        info[0] = sortKey;
        info[1] = levelTag | (((node_info->subtree_size << 4) | count) << 8) | mask;
        
        recordSize = 2 + (count+3)/4;
    } else {
        for (int i = 0; i < 8; i++) {
            if (node_info->mask & (1 << i)) {
                children[i] = node_info->sub_node_refs[i];
            } else {
                children[i] = UINT32_MAX;
            }
        }
        
        flip = self->findCanonicalOrder(children, false);
        
        uint8_t mask = self->flipMask(node_info->mask, flip);
        
        uint32_t* info_children = info + 3;
        
        count = 0;
        for (int i = 0; i < 8; i++) {
            if ((mask & (1 << i)) == 0) continue;
            info_children[count] = children[i ^ flip] ^ flip;
            count++;
        }
        
        info[0] = sortKey;
        info[1] = levelTag | (count << 8) | mask;
        info[2] = node_info->subtree_size;
        
        recordSize = 3 + count;
    }
    
    NodeRef nodeRef = self->m_buffer.size();
    self->m_buffer.insert(self->m_buffer.end(), info, info + recordSize);
    
    // If we already have an identical node, use it instead
    auto item = self->m_knownNodes.insert(nodeRef);
    if ((!item.second) || (nodeRef != *item.first)) {
        self->m_buffer.resize(nodeRef);
        nodeRef = *item.first;
        self->m_mergeCount++;
    } else {
        self->m_nodeCount++;
        
        if (level <= 2) {
            self->m_totalLeafNodes++;
            self->m_totalLeafSizes += count;
        } else {
            uint32_t* info_children = info + 3;
            for (int i = 0; i < count; i++) {
                self->reduceRarity(info_children[i] >> 3);
            }
        }
    }
    
    visitor->node_ref_new = (nodeRef << 3) | flip;
    
    return true;
}

void DAGBuilder::add(const DMirGeometry* geometry, uint32_t root) {
    DMirVisitor visitor = {.level_limit = -1};
    dmir_visit(geometry, root, &visitor, true);
    
    // Use level_limit to store the max depth of the graph
    visitor.level_limit = visitor.level_max;
    visitor.state = this;
    visitor.visit = &visit;
    dmir_visit(geometry, root, &visitor, true);
    
    m_roots.push_back(visitor.node_ref_new);
}

void DAGBuilder::build(bool compact, DMirVoxelDAG& dag, std::vector<DMirAddress>& roots) {
    std::vector<std::pair<uint32_t, uint32_t>> sorted_nodes;
    sorted_nodes.reserve(m_buffer.size());
    sorted_nodes.resize(m_knownNodes.size());
    
    const uint32_t SMALL_SUBTREE = UINT16_MAX >> 1;
    const uint32_t SMALL_ADDRESS = UINT16_MAX >> 3;
    
    uint32_t estimatedSize = 0;
    
    NodeRef ref = 0;
    uint32_t count = 0;
    while (ref < m_buffer.size()) {
        uint32_t sortKey = m_buffer[ref];
        uint32_t rarity = sortKey & DMIR_RARITY_MASK;
        uint32_t dagLevel = sortKey >> (32 - DMIR_LEVEL_BITS);
        uint32_t childCount = (m_buffer[ref+1] >> 8) & 15;
        uint32_t recordSize = (dagLevel ? 3 + childCount : 2 + (childCount+3)/4);
        
        if (dagLevel == 0) {
            estimatedSize += 2 + childCount;
        } else if (!compact) {
            estimatedSize += (2 + childCount) * 4;
        } else {
            uint32_t subtreeSize = m_buffer[ref+2];
            estimatedSize += 2 + (subtreeSize <= SMALL_SUBTREE ? 2 : 4) + childCount * 4;
        }
        
        sorted_nodes[count] = {sortKey, ref};
        
        ref += recordSize;
        count++;
    }
    
    std::sort(sorted_nodes.begin(), sorted_nodes.end());
    
    // We reuse the first pair's item as new node reference storage, so
    // we need to make sure that sorted_nodes is at least the same size
    sorted_nodes.resize(m_buffer.size());
    
    if (dag.nodes) delete[] dag.nodes;
    dag.nodes = new uint8_t[estimatedSize];
    dag.is_compact = compact;
    dag.level_count = 0;
    
    uint32_t maxSubtreeSize = 0;
    
    uint32_t pos = 0;
    
    for (uint32_t newIndex = 0; newIndex < count; newIndex++) {
        ref = sorted_nodes[newIndex].second;
        
        uint32_t sortKey = m_buffer[ref];
        uint32_t dagLevel = sortKey >> (32 - DMIR_LEVEL_BITS);
        uint32_t childCount = (m_buffer[ref+1] >> 8) & 15;
        
        if (dagLevel >= dag.level_count) {
            if (dagLevel > 0) {
                if (!compact) {
                    // All levels above 0 use 4-byte positions
                    pos += (4 - (pos & 3)) & 3;
                } else {
                    // All levels above 0 use 2-byte positions
                    pos += pos & 1;
                }
            }
            
            dag.level_starts[dagLevel] = pos;
            dag.level_count++;
        }
        
        uint32_t newRef = pos - dag.level_starts[dagLevel];
        
        if (dagLevel == 0) {
            uint8_t* node = dag.nodes + pos;
            
            uint8_t mask = m_buffer[ref+1] & 255;
            uint8_t subtreeSize = (m_buffer[ref+1] >> (4+8)) & 127;
            
            if (subtreeSize > maxSubtreeSize) maxSubtreeSize = subtreeSize;
            
            *(node++) = mask;
            
            *(node++) = subtreeSize;
            
            uint8_t* children = (uint8_t*)(m_buffer.data() + ref+2);
            for (uint32_t childIndex = 0; childIndex < childCount; childIndex++) {
                *(node++) = children[childIndex];
            }
            
            pos = node - dag.nodes;
        } else if (!compact) {
            newRef >>= 2; // convert to 4-byte position
            
            uint32_t* node = (uint32_t*)(dag.nodes + pos);
            
            uint8_t mask = m_buffer[ref+1] & 255;
            uint32_t subtreeSize = m_buffer[ref+2];
            
            if (subtreeSize > maxSubtreeSize) maxSubtreeSize = subtreeSize;
            
            *(node++) = mask;
            
            *(node++) = subtreeSize;
            
            uint32_t* children = m_buffer.data() + ref+3;
            for (uint32_t childIndex = 0; childIndex < childCount; childIndex++) {
                uint32_t oldChild = children[childIndex];
                uint32_t flip = oldChild & 7;
                oldChild >>= 3;
                uint32_t newChild = sorted_nodes[oldChild].first;
                
                *(node++) = (newChild << 3) | flip;
            }
            
            pos = ((uint8_t*)node) - dag.nodes;
        } else {
            newRef >>= 1; // convert to 2-byte position
            
            uint16_t* node = (uint16_t*)(dag.nodes + pos);
            
            uint8_t mask = m_buffer[ref+1] & 255;
            uint32_t subtreeSize = m_buffer[ref+2];
            
            if (subtreeSize > maxSubtreeSize) maxSubtreeSize = subtreeSize;
            
            uint16_t* node_mask_cell = node;
            node++;
            
            while (subtreeSize > SMALL_SUBTREE) {
                *(node++) = (1 << 15) | (subtreeSize & SMALL_SUBTREE);
                subtreeSize >>= 15;
            }
            *(node++) = subtreeSize;
            
            uint16_t refSizeMask = 0;
            uint32_t* children = m_buffer.data() + ref+3;
            for (uint32_t childIndex = 0; childIndex < childCount; childIndex++) {
                uint32_t oldChild = children[childIndex];
                uint32_t flip = oldChild & 7;
                oldChild >>= 3;
                uint32_t newChild = sorted_nodes[oldChild].first;
                
                *(node++) = ((newChild & SMALL_ADDRESS) << 3) | flip;
                
                if (newChild > SMALL_ADDRESS) {
                    refSizeMask |= 1 << childIndex;
                    *(node++) = newChild >> (16-3);
                }
            }
            
            *(node_mask_cell) = (refSizeMask << 8) | mask;
            
            pos = ((uint8_t*)node) - dag.nodes;
        }
        
        sorted_nodes[ref].first = newRef;
    }
    
    dag.level_starts[dag.level_count] = pos;
    
    m_sizeInBytes = pos;
    
    roots.clear();
    roots.reserve(m_roots.size());
    
    for (int i = 0; i < m_roots.size(); i++) {
        ref = m_roots[i];
        uint32_t flip = ref & 7;
        ref >>= 3;
        
        uint32_t sortKey = m_buffer[ref];
        uint64_t dagLevel = sortKey >> (32 - DMIR_LEVEL_BITS);
        
        uint64_t newRef = sorted_nodes[ref].first;
        
        // For correct traversal, we actually need to distinguish
        // between lower levels, so store the actual level
        uint64_t rootInfo = ((dagLevel+2) << (64-6)) | (newRef << 3) | flip;
        roots.push_back(rootInfo);
    }
}

void DAGBuilder::report() {
    std::cout << "DAG nodes: " << m_nodeCount << std::endl;
    std::cout << "Merges: " << m_mergeCount << std::endl;
    std::cout << "Voxel count: " << m_voxelCount << std::endl;
    std::cout << "Leaf nodes: " << m_totalLeafNodes << std::endl;
    std::cout << "Leaf sizes: " << m_totalLeafSizes << std::endl;
    std::cout << "Size in bytes: " << m_sizeInBytes << std::endl;
}

} // namespace dmir
