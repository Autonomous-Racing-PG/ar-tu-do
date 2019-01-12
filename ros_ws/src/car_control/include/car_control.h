#pragma once

// commands
constexpr const char* COMMAND_STOP = "stop";
constexpr const char* COMMAND_GO = "go";

// topics
constexpr const char* TOPIC_FOCBOX_SPEED = "/commands/motor/speed";
constexpr const char* TOPIC_FOCBOX_ANGLE = "/commands/servo/position";
constexpr const char* TOPIC_FOCBOX_BREAK = "commands/motor/brake";

constexpr const char* TOPIC_DRIVE_PARAM = "/set/drive_param";
constexpr const char* TOPIC_COMMAND = "/command";
constexpr const char* TOPIC_DMS = "/set/dms";