#pragma once

#include <algorithm>
#include <cstdlib>
#include <vector>
#include <eigen3/Eigen/Dense>

// vector operations
// =======================================================================================

// vector functions
// =======================================================================================

using namespace Eigen;

inline VectorXd r_uniform_real_distribution(int size)
{
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(-1, 1);
    VectorXd vec(size);
    for(int i = 0; i < size; i++)
    {
        vec[i] = dis(gen);
    }

    return vec;
}

// creates a vector with n ones and (size - n) zeros
inline VectorXd r_binary_mutation(int size, int n)
{
    VectorXd vec = VectorXd::Zero(size);
    for(int i = 0; i < size; )
    {
        
    }
    return vec;
}

inline VectorXd r_normal_distribution(int size)
{
    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::normal_distribution<> dis(-1, 1);
    VectorXd vec(size);
    for(int i = 0; i < size; i++)
    {
        vec[i] = dis(gen);
    }

    return vec;
}