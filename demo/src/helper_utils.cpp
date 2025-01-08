// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

#pragma once

#include <string>
#include <iostream>
#include <fstream>

// Some shorthands for convenience
template <typename T> using DPtr = std::unique_ptr<T>;
template <typename T> using UPtr = std::unique_ptr<T, void(*)(T*)>;

struct RGB24 {
    uint8_t r, g, b;
};

struct RGBA32 {
    uint8_t r, g, b, a;
};

void read_file(std::string path, size_t& file_size, char** data) {
    file_size = 0;
    *data = nullptr;
    
    std::ifstream file(path, std::ios::binary);
    
    if (!file) {
        std::cerr << "Failed to open file: " << path << std::endl;
        return;
    }
    
    file.seekg(0, std::ios::end);
    file_size = file.tellg();
    file.seekg(0, std::ios::beg);
    
    if (file_size > 0) {
        *data = new char[file_size];
        file.read(*data, (std::streamsize)file_size);
    } else {
        std::cerr << "Failed to seek: " << path << std::endl;
    }
    
    file.close();
}
