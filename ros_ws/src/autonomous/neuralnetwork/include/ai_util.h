#pragma once

#include "ai_math.h"

// clang-format off
#include "floatfann.h"
#include "fann_cpp.h"
// clang-format on

#include <dirent.h>
#include <string>
#include <vector>

namespace ai_util
{

    using namespace ai_math;
    using namespace std;

    inline vector<string> get_files_in_folder(string folder)
    {
        vector<string> vec;
        DIR* dir;
        struct dirent* ent;
        if ((dir = opendir(folder.c_str())) != NULL)
        {
            /* print all the files and directories within directory */
            while ((ent = readdir(dir)) != NULL)
            {
                string name = ent->d_name;
                if (name.compare(".") == 0 || name.compare("..") == 0)
                {
                    continue;
                }
                vec.push_back(folder + "/" + name);
            }
            closedir(dir);
        }
        else
        {
            /* could not open directory */
            ROS_ERROR_STREAM("could not open folder: " + folder);
        }
        return vec;
    }

    inline string net_to_string(FANN::neural_net* net)
    {
        uint n = net->get_total_neurons();
        uint c = net->get_total_connections();
        NetVector vec = net_to_vector(net);
        fann_type sig = sum(vec);
        if (sig < (fann_type)0)
        {
            sig = sig * (fann_type)-1;
        }
        while (sig < (fann_type)0.01)
        {
            sig = sig * (fann_type)10.0;
        }
        while (sig >= (fann_type)1)
        {
            sig = sig / (fann_type)10.0;
        }

        string str = "n=" + to_string(n) + " | c=" + to_string(c) + " | sig: " + to_string(sig);
        return str;
    }

    inline string net_to_string_id(FANN::neural_net* net)
    {
        NetVector vec = net_to_vector(net);
        fann_type sig = sum(vec);
        if (sig < (fann_type)0)
        {
            sig = sig * (fann_type)-1;
        }
        while (sig < (fann_type)0.01)
        {
            sig = sig * (fann_type)10.0;
        }
        while (sig >= (fann_type)1)
        {
            sig = sig / (fann_type)10.0;
        }

        string str = "" + to_string(sig);
        return str;
    }
}