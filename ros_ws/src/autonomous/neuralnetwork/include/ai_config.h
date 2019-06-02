#pragma once

/**
* @brief constants for creating networks
*/
constexpr const unsigned int NUM_LAYERS = 3;
constexpr const unsigned int NUM_INPUT = 7;
constexpr const unsigned int NUM_HIDDEN = 5;
constexpr const unsigned int NUM_OUTPUT = 2;
constexpr const unsigned int NET_ARGS[] = { NUM_INPUT, NUM_HIDDEN, NUM_OUTPUT };

constexpr const unsigned int LIDAR_INDICES[] = { 0, 1, 2, 3, 4 };