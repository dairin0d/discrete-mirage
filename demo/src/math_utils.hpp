// SPDX-License-Identifier: Zlib
// SPDX-FileCopyrightText: 2024 dairin0d https://github.com/dairin0d

extern "C" {
#include <mathc.h>
}

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

struct vec3 transform_vec3(const mfloat_t *v0, const mfloat_t *m0) {
    mfloat_t x = v0[0];
    mfloat_t y = v0[1];
    mfloat_t z = v0[2];
    return svec3(
        m0[0] * x + m0[4] * y + m0[8] * z + m0[12],
        m0[1] * x + m0[5] * y + m0[9] * z + m0[13],
        m0[2] * x + m0[6] * y + m0[10] * z + m0[14]);
}
struct vec3 transform_vec3(const struct vec3 *v0, const struct mat4 *m0) {
    return transform_vec3((mfloat_t*)v0, (mfloat_t*)m0);
}
