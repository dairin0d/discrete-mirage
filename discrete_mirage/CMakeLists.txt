add_compile_options(
    # -Ofast # can't use, since we rely on NaNs and infinities
    -O3 -fno-math-errno -fno-trapping-math
    -DCMAKE_BUILD_TYPE=Release
    -pthread
)
add_library(discrete_mirage_lib SHARED include/discrete_mirage.h src/discrete_mirage.c)
target_include_directories(discrete_mirage_lib PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
SET_TARGET_PROPERTIES(discrete_mirage_lib PROPERTIES PREFIX "")
