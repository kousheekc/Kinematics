#include "MatrixUtils.h"

MatrixUtils::MatrixUtils() {
  
}

// General matrix methods
void MatrixUtils::print_matrix(float* mat, int r, int c, String message) { 
    Serial.println(message);
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            Serial.print(mat[r * i + j]);
            Serial.print("\t");
        }
        Serial.println();
    }
    Serial.println();
}

void MatrixUtils::copy_matrix(float* mat, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[r * i + j] = mat[r * i + j];
        }
    }
}

void MatrixUtils::identity(float* mat, int n) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i == j) {
                mat[n * i + j] = 1;
            }
            else {
                mat[n * i + j] = 0;
            }
        }
    }
}

void MatrixUtils::zero(float* mat, int r, int c) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            mat[r * i + j] = 0;
        }
    }
}

void MatrixUtils::transpose(float* mat, int r, int c, float* result) {
	for (int i = 0; i < r; i++) {
		for (int j = 0; j < c; j++) {
			result[r * j + i] = mat[c * i + j];
        }
    }
}

// Transformation matrix methods
void MatrixUtils::get_rot_mat(float* mat, float* rot_mat) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rot_mat[3 * i + j] = mat[4 * i + j];
        }
    }
}

void MatrixUtils::get_pos_vec(float* mat, float* pos_vec) {
    for (int i = 0; i < 3; i++) {
        pos_vec[i] = mat[4 * i + 3];
    }
}

void MatrixUtils::create_trn_mat(float* rot_mat, float* pos_vec, float* trn_mat) {
    zero((float*)trn_mat, 4, 4);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            trn_mat[4 * i + j] = rot_mat[3 * i + j];
        }
        trn_mat[4 * i + 3] = pos_vec[i];
    }
    trn_mat[4 * 3 + 3] = 1;
}

void MatrixUtils::trn_mat_inverse(float* mat, float* result) {
    float rot_mat[3][3];
    float pos_vec[3];
    float rot_mat_t[3][3];
    float pos_vec_result[3];

    get_rot_mat((float*)mat, (float*)rot_mat);
    get_pos_vec((float*)mat, pos_vec);
    transpose((float*)rot_mat, 3, 3, (float*)rot_mat_t);
    mul_vector((float*)rot_mat_t, pos_vec, 3, 3, pos_vec_result);
    pos_vec_result[0] = -pos_vec_result[0];
    pos_vec_result[0] = -pos_vec_result[0];
    pos_vec_result[0] = -pos_vec_result[0];
    create_trn_mat((float*)rot_mat_t, pos_vec_result, (float*)result);
}

void MatrixUtils::adjoint(float* mat, float* result) {
    float rot_mat[3][3];
    float pos_vec[3];
    float so3[3][3];
    float bottom_left[3][3];

    zero((float*)result, 6, 6);
    get_rot_mat((float*)mat, (float*)rot_mat);
    get_pos_vec((float*)mat, pos_vec);
    vec_to_so3(pos_vec, (float*)so3);

    mul_matrix((float*)so3, (float*)rot_mat, 3, 3, 3, 3, (float*)bottom_left);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[6 * i + j] = rot_mat[i][j];
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[6 * (i + 3) + j] = bottom_left[i][j];
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[6 * (i + 3) + (j + 3)] = rot_mat[i][j];
        }
    }
}

void MatrixUtils::exp3(float* mat, float* result) {
    float w[3];
    float id[3][3];
    float w_mat[3][3];
    float w_mat_sq[3][3];
    float term2[3][3];
    float term3[3][3];

    identity((float*)id, 3);

    so3_to_vec((float*)mat, w);

    if (abs(norm(w)) < 1e-6) {
        copy_matrix((float*)id, 3, 3, (float*)result);
    }
    else {
        float theta = get_angle(w);

        div_scalar((float*)mat, theta, 3, 3, (float*)w_mat);
        mul_matrix((float*)w_mat, (float*)w_mat, 3, 3, 3, 3, (float*)w_mat_sq);


        mul_scalar((float*)w_mat, sin(theta), 3, 3, (float*)term2);
        mul_scalar((float*)w_mat_sq, 1 - cos(theta), 3, 3, (float*)term3);

        add_matrix((float*)id, (float*)term2, 3, 3, result);
        add_matrix((float*)result, (float*)term3, 3, 3, result);
    }
}

void MatrixUtils::exp6(float* mat, float* result) {
    float rot_mat[3][3];
    float pos_vec[3];
    float w[3];
    float id[3][3];
    float w_mat[3][3];
    float w_mat_sq[3][3];
    float result_rot[3][3];
    float result_pos_term1_a[3][3];
    float result_pos_term1_b[3][3];
    float result_pos_term1_c[3][3];
    float result_pos_term1[3][3];
    float result_pos_term2[3];
    float result_pos[3];

    identity((float*)id, 3);

    get_rot_mat((float*)mat, (float*)rot_mat);
    get_pos_vec((float*)mat, pos_vec);
    so3_to_vec((float*)rot_mat, w);

    if (abs(norm(w)) < 1e-6) {
        create_trn_mat((float*)id, pos_vec, (float*)result);
    }
    else {
        float theta = get_angle(w);

        div_scalar((float*)rot_mat, theta, 3, 3, (float*)w_mat);
        exp3((float*)rot_mat, (float*)result_rot);
        mul_matrix((float*)w_mat, (float*)w_mat, 3, 3, 3, 3, (float*)w_mat_sq);

        mul_scalar((float*)id, theta, 3, 3, (float*)result_pos_term1_a);
        mul_scalar((float*)w_mat, 1 - cos(theta), 3, 3, (float*)result_pos_term1_b);
        mul_scalar((float*)w_mat_sq, theta - sin(theta), 3, 3, (float*)result_pos_term1_c);

        add_matrix((float*)result_pos_term1_a, (float*)result_pos_term1_b, 3, 3, (float*)result_pos_term1);
        add_matrix((float*)result_pos_term1, (float*)result_pos_term1_c, 3, 3, (float*)result_pos_term1);

        div_scalar(pos_vec, theta, 1, 3, result_pos_term2);
        mul_vector((float*)result_pos_term1, result_pos_term2, 3, 3, result_pos);

        create_trn_mat((float*)result_rot, result_pos, (float*)result);
    }
}

// Vector methods
float MatrixUtils::norm(float* vec) {
    return sqrt(sq(vec[0]) + sq(vec[1]) + sq(vec[2]));
}

float MatrixUtils::get_angle(float* vec) {
    return norm(vec);
}

// Matrix operators
void MatrixUtils::add_scalar(float* mat, float s, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[r * i + j] = mat[r * i + j] + s;
        }
    }
}

void MatrixUtils::sub_scalar(float* mat, float s, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[r * i + j] = mat[r * i + j] - s;
        }
    }
}

void MatrixUtils::mul_scalar(float* mat, float s, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[r * i + j] = mat[r * i + j] * s;
        }
    }
}

void MatrixUtils::div_scalar(float* mat, float s, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[r * i + j] = mat[r * i + j] / s;
        }
    }
}

void MatrixUtils::add_matrix(float* mat1, float* mat2, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[r * i + j] = mat1[r * i + j] + mat2[r * i + j];
        }
    }
}

void MatrixUtils::sub_matrix(float* mat1, float* mat2, int r, int c, float* result) {
    for (int i = 0; i < r; i++) {
        for (int j = 0; j < c; j++) {
            result[r * i + j] = mat1[r * i + j] - mat2[r * i + j];
        }
    }
}

void MatrixUtils::mul_matrix(float* mat1, float* mat2, int r1, int c1, int r2, int c2, float* result) {
	for (int i = 0; i < r1; i++) {
		for(int j = 0; j < c2; j++) {
			result[r2 * i + j] = 0;
			for (int k = 0; k < c1; k++) {
				result[c2 * i + j] = result[c2 * i + j] + mat1[c1 * i + k] * mat2[c2 * k + j];
            }
        }
    }
}

void MatrixUtils::mul_vector(float* mat, float* vec, int r, int c, float* result) {
    for (int i = 0; i < c; i++) {
        result[i] = 0;
    }
    
    for(int i = 0; i < r; i++) {
        for(int j = 0; j < c; j++) {
            result[i] = result[i] + (vec[j] * mat[r * i + j]);
        }
    }
}

// Matrix vector methods
void MatrixUtils::vec_to_so3(float* vec, float* result) {
    result[3 * 0 + 0] = 0;
    result[3 * 0 + 1] = -vec[2];
    result[3 * 0 + 2] = vec[1];

    result[3 * 1 + 0] = vec[2];
    result[3 * 1 + 1] = 0;
    result[3 * 1 + 2] = -vec[0];

    result[3 * 2 + 0] = -vec[1];
    result[3 * 2 + 1] = vec[0];
    result[3 * 2 + 2] = 0;
}

void MatrixUtils::so3_to_vec(float* rot_mat, float* result) {
    result[0] = rot_mat[3 * 2 + 1];
    result[1] = rot_mat[3 * 0 + 2];
    result[2] = rot_mat[3 * 1 + 0];
}

void MatrixUtils::se3_to_vec(float* trn_mat, float* result) {
    result[0] = trn_mat[4 * 2 + 1];
    result[1] = trn_mat[4 * 0 + 2];
    result[2] = trn_mat[4 * 1 + 0];
    result[3] = trn_mat[4 * 0 + 3];
    result[4] = trn_mat[4 * 1 + 3];
    result[5] = trn_mat[4 * 2 + 3];
}

void MatrixUtils::vec_to_se3(float* vec, float* result) {
    float v[3] = {vec[0], vec[1], vec[2]};
    float so3_mat[3][3];

    vec_to_so3(v, (float*)so3_mat);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result[4 * i + j] = so3_mat[i][j];
        }
    }

    result[4 * 0 + 3] = vec[3];
    result[4 * 1 + 3] = vec[4];
    result[4 * 2 + 3] = vec[5];

    result[4 * 3 + 0] = 0;
    result[4 * 3 + 1] = 0;
    result[4 * 3 + 2] = 0;
    result[4 * 3 + 3] = 0;
}


