#pragma once

#include "ai_trainer.h"
#include "ai_math.h"

namespace ai_workspace
{
    using namespace ai_trainer;
    using namespace ai_math;
    using namespace std;

    // ################################################################
    // #    parameter functions
    // ################################################################
     
    // returns the fitness of the given test results m
    inline double fitness(meta* m)
    {
        double score = (m->added_velocity) / 10.0;
        return score;
    }

    // creates a mutation of a parent vector
    inline NetVector mutate(NetVector parent, double learning_rate)
    {
        
        NetVector random = r_normal_distribution(parent.size(), 0, learning_rate);
        // NetVector boolean = r_binary_mutation(parent.size(), 2);
        // random = mult(random, boolean);

        NetVector m = add(parent, random);
        return m;
    }

    // an event happend. return true to abort test
    inline bool event(meta *m, int reason)
    {
        switch(reason)
        {
            case REASON_TIMER:
            case REASON_CRASH:
            {
                m->reason = reason;
                return true;
            }
            case REASON_OUTPUT:
            {
                if (m->c_velocity < 0.0)
                {
                    m->reason = reason;
                    return true;
                }
            }

            default:
            {
                return false;
            }
        }
    }

    // ################################################################
    // #    parameter output
    // ################################################################

    inline string get_test_output(meta* m, int m_gen, int m_index, int n)
    {
        std::string str = "generation: " + std::to_string(m_gen) + " | entity: " + std::to_string(m_index) + "/" +
                        std::to_string(n)
                        + " | time: " + std::to_string(m->time) + " | score: " + std::to_string(m->score)
                        // + " | vel_sum: " +  std::to_string(m->added_velocity)
                        + " | vel_avg: " + std::to_string(m->avg_velocity) + " | abort reason: " + std::to_string(m->reason);
        return str;
    }
}