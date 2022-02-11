#ifndef UTILITIES_h
#define UTILITIES_h

#include <Arduino.h>

class Utilities {
    private:


    public:
        Utilities();

        // Matrix related methods
        float** create_mat(int n, int m=0);
        float** get_rot_mat(float** trn_mat);
        float* get_pos_vec(float** trn_mat);
        float** create_trn_mat(float** rot_mat, float* pos_vec);
        float trace(float** mat, int n);
        float** transpose(float** mat, int n);
        float** trn_mat_inverse(float** trn_mat);
        float** get_cofactor(float** mat, int p, int q, int n);
        float determinant(float** mat, int n);
        float** adj(float** mat, int n);
        float** inverse(float** mat, int n);
        float** pseudo_inverse(float** mat);
        float** zero(float** mat, int n, int m=0);
        float** adjoint(float** trn_mat);
        float** exp3(float** rot_mat);
        float** exp6(float** trn_mat);
        float** log3(float** rot_mat);
        float** log6(float** trn_mat);

        // Matrix related operators
        float** add_scalar(float** mat, float s, int n);
        float** sub_scalar(float** mat, float s, int n);
        float** mul_scalar(float** mat, float s, int n);
        float** div_scalar(float** mat, float s, int n);
        float** add_matrix(float** mat1, float** mat2, int n);
        float** sub_matrix(float** mat1, float** mat2, int n);
        float** mul_matrix(float** mat1, float** mat2, int n);
        float* mul_vector(float** mat, float* vec, int n);

        // Vector related methods
        float* create_vec(int n);
        float norm(float* pos_vec);
        float get_angle(float* pos_vec);
        float* get_axis(float* pos_vec);
        
        // Vector related operators
        float* add_scalar(float* vec, float s, int n);
        float* sub_scalar(float* vec, float s, int n);
        float* mul_scalar(float* vec, float s, int n);
        float* div_scalar(float* vec, float s, int n);
        float* add_vector(float* vec1, float* vec2, int n);
        float* mul_vector(float* vec1, float* vec2, int n);
        float dot_vector(float* vec1, float* vec2, int n);

        // Matrix vector related methods
        float** vec_to_so3(float* pos_vec);
        float* so3_to_vec(float** rot_mat);
        float** vec_to_se3(float* vel_vec);
        float* se3_to_vec(float** trn_mat);

        // Print
        void print_mat(float** mat, int n, int m=0);
        void print_vec(float* vec, int n);
};

#endif