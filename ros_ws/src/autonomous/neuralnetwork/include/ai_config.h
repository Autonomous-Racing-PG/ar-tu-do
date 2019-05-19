#pragma once

constexpr const unsigned int NUM_LAYERS = 3;
constexpr const unsigned int NUM_INPUT = 7;
constexpr const unsigned int NUM_HIDDEN = 3;
constexpr const unsigned int NUM_OUTPUT = 2;
constexpr const unsigned int NET_ARGS[] = { NUM_INPUT, NUM_HIDDEN, NUM_OUTPUT };

constexpr const unsigned int LIDAR_MIN_RANGE = 20;
constexpr const unsigned int LIDAR_MAX_RANGE = 2000;
constexpr const unsigned int LIDAR_INDICES[] = { 0, 1, 2, 3, 4 };