#pragma once

#include "ai_enum.h"
#include "ai_math.h"
#include "ai_trainer.h"

namespace ai_workspace
{
    constexpr const unsigned int NUM_LAYERS = 3;
    constexpr const unsigned int LAYER_ARRAY[] = { 7 /* number inputs*/, 5 /* hiddenlayers */, 2 /* number outputs*/ };

    using namespace ai_trainer;
    using namespace ai_math;
    using namespace ai_enum;
    using namespace std;

    // ################################################################
    // #    parameter functions
    // ################################################################

    // returns the fitness of the given test results m
    inline double fitness(TrainingContext* tc)
    {
        double score = tc->cumulative_velocity;
        return score;
    }

    // creates a mutation of a parent vector
    inline NetVector mutate(NetVector parent, double learning_rate)
    {
        auto r_uniform = sample_normal_distribution(parent.size(), 0, learning_rate);
        auto m = add(parent, r_uniform);
        return m;
    }

    // an event happend. return true to abort test
    inline bool event(TrainingContext* tc, EventCause cause)
    {
        switch (cause)
        {
            case EventCause::MAX_RUN_TIME:
            case EventCause::CRASH:
            case EventCause::LAP_FINISHED:
            {
                tc->abort_cause = cause;
                return true;
            }
            case EventCause::OUTPUT:
            default:
            {
                return false;
            }
        }
    }

    // ################################################################
    // #    parameter output
    // ################################################################

    inline string get_test_output(TrainingContext* tc, int m_gen, int m_index, int n)
    {
        std::string str = "generation: " + std::to_string(m_gen) //
            + " | entity: " + std::to_string(m_index) + "/" + std::to_string(n)
            // + " | time: " + std::to_string(m->time)
            // + " | vel_sum: " +  std::to_string(m->added_velocity)
            + " | score: " + std::to_string(tc->score) + " | vel_avg: " + std::to_string(tc->avg_velocity)
            + " | abort cause: " + ai_enum::to_string(tc->abort_cause) + " | lap_time: " + std::to_string(tc->lap_time);
        return str;
    }
}
