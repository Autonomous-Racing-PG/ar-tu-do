#pragma once

#include <string>

namespace ai_enum
{
    using namespace std;

    enum EventCause
    {
        NONE,
        CRASH,
        MAX_RUN_TIME,
        OUTPUT,
        LAP_FINISHED
    };

    inline string to_string(EventCause cause)
    {
        switch (cause)
        {
            case ai_enum::NONE:
                return "none";
            case ai_enum::CRASH:
                return "crash";
            case ai_enum::MAX_RUN_TIME:
                return "max_run_time";
            case ai_enum::OUTPUT:
                return "output";
            case ai_enum::LAP_FINISHED:
                return "lap_finished";
            default:
                return "invalid enum";
        }
    }
}