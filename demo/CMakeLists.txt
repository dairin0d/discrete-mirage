cmake_minimum_required(VERSION 3.10)

add_compile_options(
    # -Ofast
    -O3 -fno-math-errno -fno-trapping-math
    -DCMAKE_BUILD_TYPE=Release
    -pthread
)

add_executable(discrete_mirage_demo src/main.cpp mathc/mathc.c)

target_include_directories(discrete_mirage_demo PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/mathc"
    "${CMAKE_CURRENT_SOURCE_DIR}/RGFW"
)

target_link_libraries(discrete_mirage_demo PUBLIC discrete_mirage_lib)

# RGFW dependencies mentioned in https://github.com/ColleagueRiley/RGFW
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(discrete_mirage_demo PRIVATE X11 GL Xrandr m)
elseif (CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_link_libraries(discrete_mirage_demo PRIVATE opengl32 shell32 gdi32 winmm m)
elseif (CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    target_link_libraries(discrete_mirage_demo PRIVATE 
        "-framework Foundation" 
        "-framework AppKit" 
        "-framework OpenGL" 
        "-framework CoreVideo" 
        m
    )
else()
    message(FATAL_ERROR "Unsupported platform!")
endif()

add_custom_command(
    TARGET discrete_mirage_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    $<TARGET_FILE:discrete_mirage_lib>
    $<TARGET_FILE_DIR:discrete_mirage_demo>
)
