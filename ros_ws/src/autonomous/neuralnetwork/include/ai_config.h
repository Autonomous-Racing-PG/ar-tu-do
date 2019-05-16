#pragma once

constexpr const unsigned int NUM_LAYERS = 4;
constexpr const unsigned int NUM_INPUT = 7;
constexpr const unsigned int NUM_HIDDEN = 4;
constexpr const unsigned int NUM_OUTPUT = 2;
constexpr const unsigned int NET_ARGS[] = {NUM_INPUT, NUM_HIDDEN, NUM_OUTPUT};

constexpr const unsigned int LIDAR_MIN_RANGE = 20;
constexpr const unsigned int LIDAR_MAX_RANGE = 2000;
constexpr const unsigned int LIDAR_INDICES[] = { 300, 600, 900, 1200, 1500 };