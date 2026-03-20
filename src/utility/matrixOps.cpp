#include "depthai/utility/matrixOps.hpp"

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace dai {
namespace matrix {

std::vector<float> matVecMul(const std::vector<std::vector<float>>& matrix, const std::vector<float>& vec) {
    if(matrix.empty()) {
        throw std::runtime_error("Matrix should not be empty");
    }
    if(vec.empty()) {
        throw std::runtime_error("Vector should not be empty");
    }

    std::vector<float> res(matrix.size(), 0.0f);
    for(size_t i = 0; i < matrix.size(); ++i) {
        if(matrix[i].size() != vec.size()) {
            throw std::runtime_error("All matrix rows dimentsions need to match the vector size.");
        }
        for(size_t j = 0; j < matrix[0].size(); ++j) {
            res[i] += matrix[i][j] * vec[j];
        }
    }
    return res;
}

std::array<float, 3> matVecMul(const std::array<std::array<float, 3>, 3>& matrix, const std::array<float, 3>& vec) {
    std::array<float, 3> res = {0.0f, 0.0f, 0.0f};
    for(size_t i = 0; i < 3; ++i) {
        for(size_t j = 0; j < 3; ++j) {
            res[i] += matrix[i][j] * vec[j];
        }
    }
    return res;
}

bool mateq(const std::vector<std::vector<float>>& A, const std::vector<std::vector<float>>& B) {
    if(A.size() != B.size()) {
        throw std::runtime_error("Matrices have different number of rows: " + std::to_string(A.size()) + " and " + std::to_string(B.size()));
    }

    for(size_t i = 0; i < A.size(); ++i) {
        if(A[i].size() != B[i].size()) {
            throw std::runtime_error("Matrices have different number of columns in row " + std::to_string(i) + ": " + std::to_string(A[i].size()) + " and "
                                     + std::to_string(B[i].size()));
        }
        for(size_t j = 0; j < A[0].size(); ++j) {
            if(A[i][j] != B[i][j]) return false;
        }
    }
    return true;
}

bool mateq(const std::array<std::array<float, 3>, 3>& A, const std::array<std::array<float, 3>, 3>& B) {
    for(size_t i = 0; i < 3; ++i) {
        for(size_t j = 0; j < 3; ++j) {
            if(A[i][j] != B[i][j]) return false;
        }
    }
    return true;
}

std::array<std::array<float, 3>, 3> matMul(const std::array<std::array<float, 3>, 3>& A, const std::array<std::array<float, 3>, 3>& B) {
    std::array<std::array<float, 3>, 3> res = {{{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}};

    for(size_t i = 0; i < 3; ++i) {
        for(size_t j = 0; j < 3; ++j) {
            for(size_t k = 0; k < 3; ++k) {
                res[i][j] += A[i][k] * B[k][j];
            }
        }
    }

    return res;
}

std::vector<std::vector<float>> matMul(const std::vector<std::vector<float>>& firstMatrix, const std::vector<std::vector<float>>& secondMatrix) {
    if(firstMatrix.empty()) {
        throw std::runtime_error("First matrix should not be empty");
    }
    if(secondMatrix.empty()) {
        throw std::runtime_error("Second matrix should not be empty");
    }

    size_t n = firstMatrix.size();
    size_t m = firstMatrix[0].size();
    size_t p = secondMatrix.size();
    size_t q = secondMatrix[0].size();

    if(m == 0) {
        throw std::runtime_error("First matrix should not have empty rows");
    }
    for(size_t i = 1; i < firstMatrix.size(); ++i) {
        if(firstMatrix[i].size() != m) {
            throw std::runtime_error("All rows of the first matrix should have the same number of columns.");
        }
    }

    if(q == 0) {
        throw std::runtime_error("Second matrix should not have empty rows");
    }
    for(size_t i = 1; i < secondMatrix.size(); ++i) {
        if(secondMatrix[i].size() != q) {
            throw std::runtime_error("All rows of the second matrix should have the same number of columns.");
        }
    }

    if(m != p) {
        throw std::runtime_error("Internal matrix dimensions must agree. Got " + std::to_string(m) + " and " + std::to_string(p) + ".");
    }

    std::vector<std::vector<float>> res(n, std::vector<float>(q, 0.0f));

    // Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
    for(size_t i = 0; i < n; ++i) {
        for(size_t j = 0; j < q; ++j) {
            for(size_t k = 0; k < m; ++k) {
                res[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
            }
        }
    }

    return res;
}

static void getCofactor(std::vector<std::vector<float>>& A, std::vector<std::vector<float>>& temp, size_t p, size_t q, size_t n) {
    size_t i = 0, j = 0;

    // Looping for each element of the matrix
    for(size_t row = 0; row < n; ++row) {
        for(size_t col = 0; col < n; ++col) {
            //  Copying into temporary matrix only those element
            //  which are not in given row and column
            if(row != p && col != q) {
                temp[i][j++] = A[row][col];

                // Row is filled, so increase row index and
                // reset col index
                if(j == n - 1) {
                    j = 0;
                    ++i;
                }
            }
        }
    }
}

static float determinant(std::vector<std::vector<float>>& A, size_t n) {
    float D = 0;  // Initialize result

    //  Base case : if matrix contains single element
    if(n == 1) return A[0][0];

    std::vector<std::vector<float>> temp(n, std::vector<float>(n, 0));  // To store cofactors
    int sign = 1;                                                       // To store sign multiplier

    // Iterate for each element of first row
    for(size_t f = 0; f < n; ++f) {
        // Getting Cofactor of A[0][f]
        getCofactor(A, temp, 0, f, n);
        D += sign * A[0][f] * determinant(temp, n - 1);

        // terms are to be added with alternate sign
        sign = -sign;
    }

    return D;
}

static void adjoint(std::vector<std::vector<float>>& A, std::vector<std::vector<float>>& adj) {
    if(A.size() == 1) {
        adj[0][0] = 1;
        return;
    }

    // temp is used to store the final cofactors of A
    int sign = 1;
    std::vector<std::vector<float>> temp(A.size(), std::vector<float>(A.size(), 0));

    for(size_t i = 0; i < A.size(); ++i) {
        for(size_t j = 0; j < A.size(); ++j) {
            // Get cofactor of A[i][j]
            getCofactor(A, temp, i, j, A.size());

            // sign of adj[j][i] positive if sum of row
            // and column indexes is even.
            sign = ((i + j) % 2 == 0) ? 1 : -1;

            // Interchanging rows and columns to get the
            // transpose of the cofactor matrix
            adj[j][i] = (sign) * (determinant(temp, A.size() - 1));
        }
    }
}

bool matInv(std::vector<std::vector<float>>& A, std::vector<std::vector<float>>& inverse) {
    // Find determinant of A[][]
    if(A[0].size() != A.size()) {
        throw std::runtime_error("Not a Square Matrix ");
    }

    float det = determinant(A, A.size());
    if(det == 0) {
        // cout << "Singular matrix, can't find its inverse";
        return false;
    }

    // Find adjoint
    std::vector<std::vector<float>> adj(A.size(), std::vector<float>(A.size(), 0));
    adjoint(A, adj);

    std::vector<float> temp;
    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for(size_t i = 0; i < A.size(); ++i) {
        for(size_t j = 0; j < A.size(); ++j) {
            temp.push_back(adj[i][j] / det);
        }
        inverse.push_back(temp);
        temp.clear();
    }

    return true;
}

std::vector<std::vector<float>> createRotationMatrix(float theta) {
    float radian = theta * static_cast<float>(M_PI) / 180.0f;  // convert degree to radian
    std::vector<std::vector<float>> rotationMatrix = {{std::cos(radian), -std::sin(radian), 0}, {std::sin(radian), std::cos(radian), 0}, {0, 0, 1}};
    return rotationMatrix;
}

std::vector<std::vector<float>> createScalingMatrix(float scaleX, float scaleY) {
    std::vector<std::vector<float>> scalingMatrix = {{scaleX, 0, 0}, {0, scaleY, 0}, {0, 0, 1}};
    return scalingMatrix;
}

std::vector<std::vector<float>> createTranslationMatrix(float dx, float dy) {
    std::vector<std::vector<float>> translationMatrix = {{1, 0, dx}, {0, 1, dy}, {0, 0, 1}};
    return translationMatrix;
}

void printMatrix(std::vector<std::vector<float>>& matrix) {
    for(size_t i = 0; i < matrix.size(); ++i) {
        for(size_t j = 0; j < matrix[0].size(); ++j) {
            std::cout << matrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

std::vector<float> rotationMatrixToVector(const std::vector<std::vector<float>>& R) {
    return matrixToVector(R);
}

std::vector<float> matrixToVector(const std::vector<std::vector<float>>& R) {
    if(R.size() != 3 || R[0].size() != 3 || R[1].size() != 3 || R[2].size() != 3) {
        throw std::invalid_argument("Expected a 3x3 rotation matrix.");
    }

    float angle, x, y, z;

    float trace = R[0][0] + R[1][1] + R[2][2];
    float cos_angle = (trace - 1.0f) * 0.5f;

    // Clamp cos_angle to [-1, 1] to avoid NaN due to float precision
    cos_angle = std::fmax(-1.0f, std::fmin(1.0f, cos_angle));
    angle = std::acos(cos_angle);

    if(std::fabs(angle) < 1e-6f) {
        // Angle is ~0 → zero rotation vector
        return {0.0f, 0.0f, 0.0f};
    }

    float rx = R[2][1] - R[1][2];
    float ry = R[0][2] - R[2][0];
    float rz = R[1][0] - R[0][1];

    float sin_angle = std::sqrt(rx * rx + ry * ry + rz * rz) * 0.5f;

    // Normalize axis
    float k = 1.0f / (2.0f * sin_angle);
    x = k * rx;
    y = k * ry;
    z = k * rz;

    // Rotation vector = axis * angle
    return {x * angle, y * angle, z * angle};
}
std::vector<std::vector<float>> matrix3x3toVectorMatrix(const std::array<std::array<float, 3>, 3>& R) {
    std::vector<std::vector<float>> vectorR;
    for(size_t i = 0; i < 3; ++i) {
        std::vector<float> row;
        for(size_t j = 0; j < 3; ++j) {
            row.push_back(R[i][j]);
        }
        vectorR.push_back(row);
    }
    return vectorR;
}

std::vector<float> matrix3x3ToVector(const std::array<std::array<float, 3>, 3>& R) {
    std::vector<std::vector<float>> vectorR = matrix3x3toVectorMatrix(R);
    return matrixToVector(vectorR);
}

std::array<std::array<float, 3>, 3> getRotationMatrixFromProjection4x4(const std::array<std::array<float, 4>, 4>& projection) {
    std::array<std::array<float, 3>, 3> rotationMatrix = {{{projection[0][0], projection[0][1], projection[0][2]},
                                                           {projection[1][0], projection[1][1], projection[1][2]},
                                                           {projection[2][0], projection[2][1], projection[2][2]}}};
    return rotationMatrix;
}

std::vector<std::vector<float>> rvecToRotationMatrix(const double rvec[3]) {
    double theta = std::sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2]);

    std::vector<std::vector<float>> R(3, std::vector<float>(3, 0.0f));

    if(theta < 1e-10) {
        // Small angle: return identity
        R[0][0] = R[1][1] = R[2][2] = 1.0f;
        return R;
    }

    // Normalize rotation vector
    double ux = rvec[0] / theta;
    double uy = rvec[1] / theta;
    double uz = rvec[2] / theta;

    double c = std::cos(theta);
    double s = std::sin(theta);
    double one_minus_c = 1.0 - c;

    R[0][0] = c + ux * ux * one_minus_c;
    R[0][1] = ux * uy * one_minus_c - uz * s;
    R[0][2] = ux * uz * one_minus_c + uy * s;

    R[1][0] = uy * ux * one_minus_c + uz * s;
    R[1][1] = c + uy * uy * one_minus_c;
    R[1][2] = uy * uz * one_minus_c - ux * s;

    R[2][0] = uz * ux * one_minus_c - uy * s;
    R[2][1] = uz * uy * one_minus_c + ux * s;
    R[2][2] = c + uz * uz * one_minus_c;

    return R;
}

std::array<std::array<float, 2>, 2> getMatrixInverse(const std::array<std::array<float, 2>, 2>& matrix) {
    auto det = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    if(det == 0) {
        throw std::runtime_error("Determinant is zero");
    }
    return {{{matrix[1][1] / det, -matrix[0][1] / det}, {-matrix[1][0] / det, matrix[0][0] / det}}};
}

std::array<std::array<float, 3>, 3> getMatrixInverse(const std::array<std::array<float, 3>, 3>& matrix_float) {
    // Step 1: Convert to double
    std::array<std::array<double, 3>, 3> matrix;
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j) matrix[i][j] = static_cast<double>(matrix_float[i][j]);

    std::array<std::array<float, 3>, 3> inv;
    double det = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
                 - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
                 + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);

    if(det == 0) {
        throw std::runtime_error("Matrix is singular and cannot be inverted.");
    }

    std::array<std::array<double, 3>, 3> adj;

    adj[0][0] = (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]);
    adj[0][1] = -(matrix[0][1] * matrix[2][2] - matrix[0][2] * matrix[2][1]);
    adj[0][2] = (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]);

    adj[1][0] = -(matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0]);
    adj[1][1] = (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]);
    adj[1][2] = -(matrix[0][0] * matrix[1][2] - matrix[0][2] * matrix[1][0]);

    adj[2][0] = (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    adj[2][1] = -(matrix[0][0] * matrix[2][1] - matrix[0][1] * matrix[2][0]);
    adj[2][2] = (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]);

    double invDet = 1.0 / det;

    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            inv[i][j] = static_cast<float>(adj[i][j] * invDet);
        }
    }

    return inv;
}

void invertSe3Matrix4x4InPlace(std::vector<std::vector<float>>& mat) {
    if(mat.size() != 4) {
        throw std::invalid_argument("Expected a 4x4 matrix.");
    }
    for(const auto& row : mat) {
        if(row.size() != 4) {
            throw std::invalid_argument("Expected a 4x4 matrix.");
        }
    }

    const float r00 = mat[0][0], r01 = mat[0][1], r02 = mat[0][2];
    const float r10 = mat[1][0], r11 = mat[1][1], r12 = mat[1][2];
    const float r20 = mat[2][0], r21 = mat[2][1], r22 = mat[2][2];

    const float tx = mat[0][3], ty = mat[1][3], tz = mat[2][3];

    // t' = -R^T t
    mat[0][3] = -(r00 * tx + r10 * ty + r20 * tz);
    mat[1][3] = -(r01 * tx + r11 * ty + r21 * tz);
    mat[2][3] = -(r02 * tx + r12 * ty + r22 * tz);

    // R' = R^T
    mat[0][0] = r00;
    mat[0][1] = r10;
    mat[0][2] = r20;
    mat[1][0] = r01;
    mat[1][1] = r11;
    mat[1][2] = r21;
    mat[2][0] = r02;
    mat[2][1] = r12;
    mat[2][2] = r22;

    mat[3][0] = 0.0f;
    mat[3][1] = 0.0f;
    mat[3][2] = 0.0f;
    mat[3][3] = 1.0f;
}
std::vector<std::vector<float>> invertSe3Matrix4x4(const std::vector<std::vector<float>>& matrix) {
    if(matrix.size() != 4 || matrix[0].size() != 4 || matrix[1].size() != 4 || matrix[2].size() != 4 || matrix[3].size() != 4) {
        throw std::invalid_argument("Expected a 4x4 matrix.");
    }

    std::vector<std::vector<float>> inv(4, std::vector<float>(4, 0.0f));

    // Transpose rotation part (R^T)
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            inv[i][j] = matrix[j][i];
        }
    }

    // Invert translation: -R^T * t
    for(int i = 0; i < 3; ++i) {
        float newTrans = 0.0f;
        for(int j = 0; j < 3; ++j) {
            newTrans -= inv[i][j] * matrix[j][3];
        }
        inv[i][3] = newTrans;
    }

    inv[3][3] = 1.0f;
    return inv;
}
}  // namespace matrix
}  // namespace dai
