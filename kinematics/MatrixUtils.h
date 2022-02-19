#ifndef MATRIXUTILS_h
#define MATRIXUTILS_h

#include <Arduino.h>

class MatrixUtils {
    private:

    public:
        MatrixUtils();

        void print_matrix(float* mat, int r, int c, String message="");
        void copy_matrix(float* mat, int r, int c, float* result);

        void identity(float* mat, int n);
        void zero(float* mat, int r, int c);
        void get_rot_mat(float* mat, float* rot_mat);
        void get_pos_vec(float* mat, float* pos_vec);
        void create_trn_mat(float* rot_mat, float* pos_vec, float* trn_mat);
        void adjoint(float* mat, float* result);
        void trn_mat_inverse(float* mat, float* result);
        void transpose(float* mat, int r, int c, float* result);

        float norm(float* vec);
        float get_angle(float* vec);
        
        void add_scalar(float* mat, float s, int r, int c, float* result);
        void sub_scalar(float* mat, float s, int r, int c, float* result);
        void mul_scalar(float* mat, float s, int r, int c, float* result);
        void div_scalar(float* mat, float s, int r, int c, float* result);

        void add_matrix(float* mat1, float* mat2, int r, int c, float* result);
        void sub_matrix(float* mat1, float* mat2, int r, int c, float* result);
        void mul_matrix(float* mat1, float* mat2, int r1, int c1, int r2, int c2, float* result);
        void mul_vector(float* mat1, float* vec, int r, int c, float* result);

        void vec_to_so3(float* vec, float* result);
        void so3_to_vec(float* rot_mat, float* result);
        void vec_to_se3(float* vec, float* result);
        void se3_to_vec(float* trn_mat, float* result);

        void exp3(float* mat, float* result);
        void exp6(float* mat, float* result);
};

#endif
