#pragma once

namespace ai_config
{
    /**
    * @brief constants for creating networks
    */
    constexpr const unsigned int DEFAULT_NUM_LAYERS = 3;
    constexpr const unsigned int DEFAULT_NUM_INPUT = 7;
    constexpr const unsigned int DEFAULT_NUM_HIDDEN = 5;
    constexpr const unsigned int DEFAULTNUM_OUTPUT = 2;
    constexpr const unsigned int DEFAULT_LAYER_ARRAY[] = { DEFAULT_NUM_INPUT, DEFAULT_NUM_HIDDEN, DEFAULTNUM_OUTPUT };
}