#include "Utilities.h"

Utilities::Utilities() {

}
// Matrix related methods
float** Utilities::create_mat(int n, int m) {
    if (m == 0) {
        m = n;
    }

    float** result = new float*[n];

    for (int i = 0; i < n; i++) {
        result[i] = new float[m];
        for (int j = 0; j < m; j++) {
            if (i == j) {
                result[i][j] = 1;
            }
            else {
                result[i][j] = 0;
            }
        }
    }

    return result;
}

float** Utilities::get_rot_mat(float** trn_mat) {
    float** result = create_mat(3);
    
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = trn_mat[i][j];
        }
    }

    return result;
}

float* Utilities::get_pos_vec(float** trn_mat) {
    float* result = new float[3];
    
    for (int i = 0; i < 3; i++) {
        result[i] = trn_mat[i][3];
    }

    return result;
}

float** Utilities::create_trn_mat(float** rot_mat, float* pos_vec) {
    float** result = create_mat(4);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = rot_mat[i][j];
        }
        result[i][3] = pos_vec[i];
    }

    return result;
}

float Utilities::trace(float** mat, int n) {
    float sum = 0;
    for (int i = 0; i < n; i++) {
        sum += mat[i][i]; 
    }
    return sum;
}

float** Utilities::transpose(float** mat, int n) {
    float** result = create_mat(n);
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            result[i][j] = mat[j][i];
        }
    }
    return result;
}

float** Utilities::trn_mat_inverse(float** trn_mat) {
    float** r = get_rot_mat(trn_mat);
    float* p = get_pos_vec(trn_mat);
    float** r_result = transpose(r, 3);
    float* p_result = mul_vector(r_result, p, 3);
    for (int i = 0; i < 3; i++) {
        p_result[i] = -p_result[i];
    }
    return create_trn_mat(r_result, p_result);
}

float** Utilities::get_cofactor(float** mat, int p, int q, int n) {
    float** result = create_mat(n-1);
    int row = 0;
    int col = 0;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i != p && j != q) {
                result[row][col] = mat[i][j];
                col++;
                if (col == n-1) {
                    col = 0;
                    row++;
                }
            }
        }
    }

    return result;
}

float Utilities::determinant(float** mat, int n) {
    float d = 0;
    if (n == 1) {
        return mat[0][0];
    }
    int sign = 1;
    for (int i = 0; i < n; i++) {
        float** temp = get_cofactor(mat, 0, i, n);
        d += sign * mat[0][i] * determinant(temp, n-1);
        sign = -sign;
    }
    return d;
}

float** Utilities::adj(float** mat, int n) {
    float** result = create_mat(n);
    if (n == 1) {
        result[0][0] = 1;
        return result;
    }
    
    int sign = 1;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            float** temp = get_cofactor(mat, i, j, n);
            sign = ((i + j) % 2 == 0) ? 1 : -1;
            result[i][j] = (sign) * (determinant(temp, n-1));
        }
    }
    return result;
}

float** Utilities::inverse(float** mat, int n) {
    float det = determinant(mat, n);
    if (det == 0) {
        Serial.println("Singular matrix");
        return NULL;
    }
    float** adj = adj(mat, n);
    float** result = div_scalar(adj, det, n);
    return result;
}

float** Utilities::pseudo_inverse(float** mat) {

}

float** Utilities::zero(float** mat, int n, int m) {
    if (m == 0) {
        m = n;
    }
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            mat[i][j] = 0;
        }
    }
    return mat;
}

float** Utilities::adjoint(float** trn_mat) {
    float** result = create_mat(6);

    float** rot_mat = get_rot_mat(trn_mat);
    float** so3 = vec_to_se3(get_pos_vec(trn_mat));

    float** bottom_left = mul_matrix(so3, rot_mat, 3);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = rot_mat[i][j];
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i+3][j] = bottom_left[i][j];
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i+3][j+3] = rot_mat[i][j];
        }
    }

    return result;
}

float** Utilities::exp3(float** rot_mat) {
    float** result = create_mat(3);

    float* w = so3_to_vec(rot_mat);

    if (abs(norm(w)) < 1e-6) {
        return result;
    }
    else {
        float theta = get_angle(w);
        float** w_mat = div_scalar(rot_mat, theta, 3);

        float** term1 = create_mat(3);
        float** term2 = mul_scalar(w_mat, sin(theta), 3);
        float** term3 = mul_scalar(mul_matrix(w_mat, w_mat, 3), 1 - cos(theta), 3);

        result = add_matrix(term1, add_matrix(term2, term3, 3), 3);
    }
}

float** Utilities::exp6(float** trn_mat) {
    float* w = so3_to_vec(get_rot_mat(trn_mat));

    if (abs(norm(w)) < 1e-6) {
        return create_trn_mat(create_mat(3), get_pos_vec(trn_mat));
    }
    else{
        float theta = get_angle(w);
        float** w_mat = div_scalar(get_rot_mat(trn_mat), theta, 3);
        
        float** result_rot = exp3(get_rot_mat(trn_mat));

        float** result_pos_term1_a = mul_scalar(create_mat(3), theta, 3);
        float** result_pos_term1_b = mul_scalar(w_mat, 1 - cos(theta), 3);
        float** result_pos_term1_c = mul_scalar(mul_matrix(w_mat, w_mat, 3), theta - sin(theta), 3);
        
        float** result_pos_term1 = add_matrix(result_pos_term1_a, add_matrix(result_pos_term1_b, result_pos_term1_c, 3), 3);
        float* result_pos_term2 = div_scalar(get_pos_vec(trn_mat), theta, 3);

        float* result_pos = mul_vector(result_pos_term1, result_pos_term2, 3);
        
        return create_trn_mat(result_rot, result_pos);
    }
}

float** Utilities::log3(float** rot_mat) {
    float acos_input = (trace(rot_mat, 3) - 1.0) / 2.0;

    if (acos_input >= 1) {
        float** result = create_mat(3);
        result = zero(result, 3);
        return result;
    }
    else if (acos_input <= -1) {
        float* w = create_vec(3);
        float s;
        
        if (1 + rot_mat[2][2] > 1e-6) {
            w[0] = rot_mat[0][2];
            w[1] = rot_mat[1][2];
            w[2] = 1 + rot_mat[2][2];
            s = 1.0 / sqrt(2.0 * (1 + rot_mat[2][2]));
            w = mul_scalar(w, s, 3);
        }
        else if (1 + rot_mat[1][1] >= 1e-6) {
            w[0] = rot_mat[0][1];
            w[1] = 1 + rot_mat[1][1];
            w[2] = rot_mat[2][1];
            s = 1.0 / sqrt(2.0 * (1 + rot_mat[1][1]));
            w = mul_scalar(w, s, 3);
        }
        else {
            w[0] = 1 + rot_mat[0][0];
            w[1] = rot_mat[1][0];
            w[2] = rot_mat[2][0];
            s = 1.0 / sqrt(2.0 * (1 + rot_mat[0][0]));
            w = mul_scalar(w, s, 3);
        }
        return vec_to_so3(mul_scalar(w, PI, 3));
    }
    else {
        float theta = acos(acos_input);
        float s = theta / 2.0 / sin(theta);
        float** term = sub_matrix(rot_mat, transpose(rot_mat, 3), 3);
        return mul_scalar(term, s, 3);
    }
}

float** Utilities::log6(float** trn_mat) {
    float** rot_mat = get_rot_mat(trn_mat);
    float** w_mat = log3(rot_mat);
    bool condition = true;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (w_mat[i][j] != 0) {
                condition = false;
            }
        }
    }
    if (condition == true) {
        float** rot = zero(create_mat(3), 3);
        float* vec = create_vec(3);
        vec[0] = trn_mat[0][3];
        vec[1] = trn_mat[1][3];
        vec[2] = trn_mat[2][3];

        float** result = create_trn_mat(rot, vec);
        result[3][3] = 0;
        return result;
    }
    else {
        float theta = acos((trace(rot_mat, 3) - 1.0) / 2.0);
        float** term1 = sub_matrix(create_mat(3), div_scalar(w_mat, 2.0, 3), 3);
        float** term2 = mul_scalar(div_scalar(mul_matrix(w_mat, w_mat, 3), theta, 3), (1.0 / theta - 1.0 / tan(theta / 2.0) / 2.0), 3);
        
        float* term3 = create_vec(3);
        term3[0] = trn_mat[0][3];
        term3[1] = trn_mat[1][3];
        term3[2] = trn_mat[2][3];

        print_mat(term2, 3);
        Serial.println();

        float* p = mul_vector(add_matrix(term1, term2, 3), term3, 3);

        float** result = create_trn_mat(w_mat, p);
        result[3][3] = 0;
        return result;
    }
}

// Matrix related operators
float** Utilities::add_scalar(float** mat, float s, int n) {
    float** result = create_mat(n);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            result[i][j] = mat[i][j] + s;
        }
    }

    return result;
}

float** Utilities::sub_scalar(float** mat, float s, int n) {
    float** result = create_mat(n);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            result[i][j] = mat[i][j] - s;
        }
    }

    return result;
}

float** Utilities::mul_scalar(float** mat, float s, int n) {
    float** result = create_mat(n);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            result[i][j] = mat[i][j] * s;
        }
    }

    return result;
}

float** Utilities::div_scalar(float** mat, float s, int n) {
    float** result = create_mat(n);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            result[i][j] = mat[i][j] / s;
        }
    }

    return result;
}

float** Utilities::add_matrix(float** mat1, float** mat2, int n) {
    float** result = create_mat(n);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            result[i][j] = mat1[i][j] + mat2[i][j];
        }
    }

    return result;
}

float** Utilities::sub_matrix(float** mat1, float** mat2, int n) {
    float** result = create_mat(n);

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            result[i][j] = mat1[i][j] - mat2[i][j];
        }
    }

    return result;
}

float** Utilities::mul_matrix(float** mat1, float** mat2, int n) {
    float** result = create_mat(n);

    for (int i = 0; i < n; i++) {
        result[i][i] = 0;
    }
    
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            for (int k = 0; k < n; k++) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }

    return result;
}

float* Utilities::mul_vector(float** mat, float* vec, int n) {
    float* result = new float[n];

    for (int i = 0; i < n; i++) {
        result[i] = 0;
    }
    
    for(int i = 0; i < n; i++) {
        for(int j = 0; j < n; j++) {
            result[i] = result[i] + (vec[j] * mat[i][j]);
        }
    }

    return result;
}

// Vector related methods
float* Utilities::create_vec(int n) {
    float* result = new float[n];
    for (int i = 0; i < n; i++) {
        result[i] = 0;
    }
    return result;
}

float Utilities::norm(float* pos_vec) {
    return sqrt(sq(pos_vec[0]) + sq(pos_vec[1]) + sq(pos_vec[2]));
}

float Utilities::get_angle(float* pos_vec) {
    return norm(pos_vec);
}

float* Utilities::get_axis(float* pos_vec) {
    float* result = new float[3];

    float n = norm(pos_vec);
    result[0] = pos_vec[0]/n;
    result[1] = pos_vec[1]/n;
    result[2] = pos_vec[2]/n;

    return result;
}

// Vector related operators
float* Utilities::add_scalar(float* vec, float s, int n) {
    float* result = new float[n];

    for (int i = 0; i < n; i++) {
        result[i] = vec[i] + s;
    }

    return result;
}

float* Utilities::sub_scalar(float* vec, float s, int n) {
    float* result = new float[n];

    for (int i = 0; i < n; i++) {
        result[i] = vec[i] - s;
    }

    return result;
}

float* Utilities::mul_scalar(float* vec, float s, int n) {
    float* result = new float[n];

    for (int i = 0; i < n; i++) {
        result[i] = vec[i] * s;
    }

    return result;
}

float* Utilities::div_scalar(float* vec, float s, int n) {
    float* result = new float[n];

    for (int i = 0; i < n; i++) {
        result[i] = vec[i] / s;
    }

    return result;
}

float* Utilities::add_vector(float* vec1, float* vec2, int n) {
    float* result = create_vec(n);
    for (int i = 0; i < n; i++) {
        result[i] = vec1[i] + vec2[i];
    }
    return result;
}


float* Utilities::mul_vector(float* vec1, float* vec2, int n) {
    float* result = create_vec(n);
    for (int i = 0; i < n; i++) {
        result[i] = vec1[i] * vec2[i];
    }
    return result;
}

float Utilities::dot_vector(float* vec1, float* vec2, int n) {
    float result = 0;
    for (int i = 0; i < n; i++) {
        result += vec1[i] * vec2[i];
    }
    return result;
}

// Matrix vector related methods
float** Utilities::vec_to_so3(float* pos_vec) {
    float** result = create_mat(3);

    result[0][0] = 0;
    result[0][1] = -pos_vec[2];
    result[0][2] = pos_vec[1];

    result[1][0] = pos_vec[2];
    result[1][1] = 0;
    result[1][2] = -pos_vec[0];

    result[2][0] = -pos_vec[1];
    result[2][1] = pos_vec[0];
    result[2][2] = 0;

    return result;
}

float* Utilities::so3_to_vec(float** rot_mat) {
    float* result = new float[3];

    result[0] = rot_mat[2][1];
    result[1] = rot_mat[0][2];
    result[2] = rot_mat[1][0];
    
    return result;
}

float** Utilities::vec_to_se3(float* vel_vec) {
    float** result = create_mat(4);
    
    float v1[3] = {vel_vec[0], vel_vec[1], vel_vec[2]};
    float** so3 = vec_to_so3(v1);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = so3[i][j];
        }
    }
    result[0][3] = vel_vec[3];
    result[1][3] = vel_vec[4];
    result[2][3] = vel_vec[5];

    result[3][3] = 0;

    return result;
}

float* Utilities::se3_to_vec(float** trn_mat) {
    float* result = new float[6];

    result[0] = trn_mat[2][1];
    result[1] = trn_mat[0][2];
    result[2] = trn_mat[1][0];
    result[3] = trn_mat[0][3];
    result[4] = trn_mat[1][3];
    result[5] = trn_mat[2][3];

    return result;
}

// Print
void Utilities::print_mat(float** mat, int n, int m) {
    if (m == 0) {
        m = n;
    }
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            Serial.print(mat[i][j]);
            Serial.print("\t");
        }
        Serial.println();
    }
}

void Utilities::print_vec(float* vec, int n) {
    for (int i = 0; i < n; i++) {
        Serial.print(vec[i]);
        Serial.print("\t");
    }
    Serial.println();
}
