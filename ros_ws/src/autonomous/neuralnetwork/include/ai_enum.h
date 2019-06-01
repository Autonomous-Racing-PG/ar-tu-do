#pragma once

#include <string>

namespace ai_enum
{
    using namespace std;

    enum AbortReason
    {
        none,
        crash,
        max_run_time,
        output,
        lap_finished
    };

    inline string to_string(AbortReason reason)
    {
        switch (reason)
        {
        case ai_enum::none:
            return "none";
        case ai_enum::crash:
            return "crash";
        case ai_enum::max_run_time:
            return "max_run_time";
        case ai_enum::output:
            return "output";
        case ai_enum::lap_finished:
            return "lap_finished";
        default:
            return "invalid enum";
        }
    }
}