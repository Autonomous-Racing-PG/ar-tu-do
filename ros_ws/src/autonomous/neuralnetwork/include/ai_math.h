#pragma once

#include <algorithm>
#include <cstdlib>
#include <vector>

// clang-format off
#include "floatfann.h"
#include "fann_cpp.h"
// clang-format on

namespace ai_math
{
    typedef std::vector<fann_type> NetVector;
    
    class Singleton
    {
    private:
		static bool instanceFlag;
		static Singleton *single;
		Singleton()
		{
			//private constructor
            std::random_device rd;  // Will be used to obtain a seed for the random number engine
            std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
            random_generator = gen;
		}

    public:
		static Singleton* getInstance();
        std::mt19937 random_generator;
		~Singleton()
		{
			instanceFlag = false;
		}
    };

    bool Singleton::instanceFlag = false;
    Singleton* Singleton::single = NULL;
    
    Singleton* Singleton::getInstance()
    {
        if (!instanceFlag)
        {
            single = new Singleton();
            instanceFlag = true;
            return single;
        }
        else
        {
            return single;
        }
    }

    // ################################################################
    // #    create vectors
    // ################################################################

    // returns a vector with random values between min and max
    inline NetVector sample_uniform_distribution(uint size, fann_type min = -1, fann_type max = 1)
    {
        if (min > max)
        {
            // switch min and max
            double tmp = min;
            min = max;
            max = tmp;
        }

        std::uniform_real_distribution<fann_type> dis(min, max);
        NetVector vec(size);
        for (uint i = 0; i < size; i++)
        {
            vec[i] = dis(Singleton::getInstance()->random_generator);
        }
        return vec;
    }

    // returns a vector with random values between min and max
    inline NetVector sample_normal_distribution(uint size, double mean = 0, double stddev = 0.5)
    {
        std::normal_distribution<fann_type> dis(mean, stddev);
        NetVector vec(size);
        for (uint i = 0; i < size; i++)
        {
            vec[i] = dis(Singleton::getInstance()->random_generator);
        }
        return vec;
    }

    // creates a vector with n ones and (size - n) zeros
    inline NetVector sample_binary_permutation(uint size, uint number_of_ones = 1)
    {
        assert(number_of_ones <= size);
        NetVector vec(size);
        for(uint i = 0; i < size; i++)
        {
            if(i < number_of_ones)
            {
                vec[i] = 1.0;
            }
            else 
            {
                vec[i] = 0.0;
            }
        }
        std::shuffle(vec.begin(), vec.end(), Singleton::getInstance()->random_generator);
        return vec;
    }

    inline NetVector zeros(uint size)
    {
        NetVector vec(size);
        for (uint i = 0; i < size; i++)
        {
            vec[i] = 0.0;
        }
        return vec;
    }

    inline NetVector ones(uint size)
    {
        NetVector vec(size);
        for (uint i = 0; i < size; i++)
        {
            vec[i] = 1.0;
        }
        return vec;
    }

    inline NetVector clone(NetVector& a)
    {
        NetVector c(a.size());
        for (uint i = 0; i < a.size(); i++)
        {
            c[i] = a[i];
        }
        return c;
    }

    // ################################################################
    // #    vector manupulation
    // ################################################################

    inline NetVector mult(NetVector& a, NetVector& b)
    {
        assert(a.size() == b.size());
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
        assert(a.size() == b.size());
        NetVector c(a.size());
        for (uint i = 0; i < a.size(); i++)
        {
            c[i] = a[i] + b[i];
        }
        return c;
    }

    inline NetVector add(NetVector& a, fann_type b)
    {
        NetVector c(a.size());
        for (uint i = 0; i < a.size(); i++)
        {
            c[i] = a[i] + b;
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

    // switches random pairs of values
    inline NetVector mutation_exchange(NetVector& input, uint number_of_swaps)
    {
        uint size = input.size();
        std::uniform_int_distribution<> dis(0, size - 1);

        NetVector result = clone(input);
        for (uint i = 0; i < number_of_swaps; i++)
        {
            int swap_index_1;
            int swap_index_2;
            do
            {
                swap_index_1 = dis(Singleton::getInstance()->random_generator);
                swap_index_2 = dis(Singleton::getInstance()->random_generator);
            } while (swap_index_1 == swap_index_2);

            fann_type tmp = result[swap_index_1];
            result[swap_index_1] = result[swap_index_2];
            result[swap_index_2] = tmp;
        }
        return result;
    }

    // adds a uniform random vector with mean = 0 and stddev = learning_rate to p
    inline NetVector mutation_add_uniform(NetVector& p, double learning_rate)
    {
        NetVector random = sample_normal_distribution(p.size(), 0, learning_rate);
        NetVector m = add(p, random);
        return m;
    }

    // mults a uniform random vector with mean = 0 and stddev = learning_rate to p
    inline NetVector mutation_mult_uniform(NetVector& p, double learning_rate)
    {
        NetVector random = sample_normal_distribution(p.size(), 1, learning_rate);
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

    inline FANN::neural_net* vector_to_net(NetVector& vec, uint layers, uint* layer_array)
    {
        FANN::neural_net* net = new FANN::neural_net();
        net->create_standard_array(layers, layer_array);
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
