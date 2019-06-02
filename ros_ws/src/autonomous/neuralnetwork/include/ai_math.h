#pragma once

#include <algorithm>
#include <cstdlib>
#include <vector>

#include "ros/static_assert.h"

// http://leenissen.dk/fann/html/files/fann_cpp-h.html
// clang-format off
#include "floatfann.h"
#include "fann_cpp.h"
// clang-format on

namespace ai_math
{
    using namespace std;

    typedef std::vector<fann_type> NetVector;

    // ################################################################
    // #    create vectors
    // ################################################################

    // returns a vector with random values between min and max
    inline NetVector r_uniform_distribution(int size, fann_type min = -1, fann_type max = 1)
    {
        if (min > max)
        {
            // switch min and max
            double tmp = min;
            min = max;
            max = tmp;
        }

        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<fann_type> dis(min, max);
        NetVector vec;
        for (int i = 0; i < size; i++)
        {
            vec.push_back(dis(gen));
        }

        return vec;
    }

    // returns a vector with random values between min and max
    inline NetVector r_normal_distribution(uint size, double mean = 0, double stddev = 0.5)
    {
        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        std::normal_distribution<fann_type> dis(mean, stddev);
        NetVector vec;
        for (uint i = 0; i < size; i++)
        {
            vec.push_back(dis(gen));
        }

        return vec;
    }

    // creates a vector with n ones and (size - n) zeros
    inline NetVector r_binary_mutation(uint size, int n = 1)
    {
        std::vector<bool> boolean;
        for (uint i = 0; i < size; i++)
        {
            boolean.push_back(false);
        }

        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        std::uniform_int_distribution<> dis(0, size - 1);

        int ones = 0;
        while (ones != n)
        {
            int target = dis(gen);
            if (boolean[target] == false)
            {
                boolean[target] = true;
                ones++;
            }
        }

        NetVector vec(size);
        for (uint i = 0; i < size; i++)
        {
            vec[i] = boolean[i] ? 1.0 : 0.0;
        }

        return vec;
    }

    inline NetVector zeros(uint size)
    {
        NetVector vec;
        for (uint i = 0; i < size; i++)
        {
            vec.push_back(0);
        }
        return vec;
    }

    inline NetVector ones(uint size)
    {
        NetVector vec;
        for (uint i = 0; i < size; i++)
        {
            vec.push_back(1);
        }
        return vec;
    }

    inline NetVector clone(NetVector& a)
    {
        NetVector c;
        for (uint i = 0; i < a.size(); i++)
        {
            c.push_back(a[i]);
        }
        return c;
    }

    // ################################################################
    // #    vector manupulation
    // ################################################################

    inline NetVector mult(NetVector& a, NetVector& b)
    {
        NetVector c(a.size());
        for (uint i = 0; i < a.size(); i++)
        {
            c[i] = a[i] * b[i];
        }
        return c;
    }

    inline NetVector mult(NetVector& a, fann_type scalar)
    {
        NetVector c(a.size());
        for (uint i = 0; i < a.size(); i++)
        {
            c[i] = a[i] * scalar;
        }
        return c;
    }

    inline NetVector add(NetVector& a, NetVector& b)
    {
        NetVector c(a.size());
        for (uint i = 0; i < a.size(); i++)
        {
            c[i] = a[i] + b[i];
        }
        return c;
    }

    inline NetVector add(NetVector& a, fann_type add)
    {
        NetVector c(a.size());
        for (uint i = 0; i < a.size(); i++)
        {
            c[i] = a[i] + add;
        }
        return c;
    }

    inline fann_type sum(NetVector& a)
    {
        fann_type n = 0;
        for (uint i = 0; i < a.size(); i++)
        {
            n = n + a[i];
        }
        return n;
    }

    // ################################################################
    // #    vector mutation
    // ################################################################

    inline NetVector m_exchange_mutation(NetVector& p, uint switches)
    {
        uint size = p.size();

        std::random_device rd;  // Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
        std::uniform_int_distribution<> dis(0, size - 1);

        NetVector m = clone(p);
        for (uint i = 0; i < switches; i++)
        {
            int a;
            int b;
            do
            {
                a = dis(gen);
                b = dis(gen);
            } while (a != b);

            fann_type tmp = m[a];
            m[a] = m[b];
            m[b] = tmp;
        }

        return m;
    }

    inline NetVector m_uniform_add_mutation(NetVector& p, double learning_rate)
    {
        NetVector random = r_normal_distribution(p.size(), 0, learning_rate);
        NetVector m = add(p, random);
        return m;
    }

    inline NetVector m_uniform_mult_mutation(NetVector& p, double learning_rate)
    {
        NetVector random = r_normal_distribution(p.size(), 1, learning_rate);
        NetVector m = mult(p, random);
        return m;
    }

    // ################################################################
    // #    vector conversion
    // ################################################################

    inline std::string to_string(NetVector& vec)
    {
        std::string str = "";
        for (uint i = 0; i < vec.size(); i++)
        {
            str = str + std::to_string(vec[i]) + " ";
        }
        return str;
    }

    inline NetVector net_to_vector(FANN::neural_net* net)
    {
        uint size = net->get_total_connections();
        FANN::connection arr[size];
        net->get_connection_array(arr);

        NetVector vec(size);
        for (uint i = 0; i < size; i++)
        {
            vec[i] = arr[i].weight;
        }
        return vec;
    }

    inline FANN::neural_net* vector_to_net(NetVector& vec)
    {
        FANN::neural_net* net = new FANN::neural_net();
        net->create_standard_array(NUM_LAYERS, NET_ARGS);
        uint size = net->get_total_connections();
        FANN::connection arr[size];
        net->get_connection_array(arr);
        for (uint i = 0; i < size; i++)
        {
            arr[i].weight = vec[i];
        }
        net->set_weight_array(arr, size);
        return net;
    }
}
