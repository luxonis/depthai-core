#pragma once

#include <vector>

std::vector<std::vector<float>> mat_mul(std::vector<std::vector<float>>& firstMatrix, 
                                        std::vector<std::vector<float>>& secondMatrix);

bool mat_inv(std::vector<std::vector<float>>& A, std::vector<std::vector<float>>& inverse);


void LU_decomp(
            std::vector<std::vector<float>>& input_matrix, 
            std::vector<std::vector<float>>& l_matrix, 
            std::vector<std::vector<float>>& u_matrix);