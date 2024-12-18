cmake_minimum_required(VERSION 3.10)

add_compile_options(
    # -Ofast
    -O3 -fno-math-errno -fno-trapping-math
    -DCMAKE_BUILD_TYPE=Release
    -pthread
)
add_executable(discrete_mirage_demo src/main.cpp mathc/mathc.c)
target_include_directories(discrete_mirage_demo PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/mathc")
target_link_libraries(discrete_mirage_demo PUBLIC discrete_mirage_lib SDL2main SDL2)
