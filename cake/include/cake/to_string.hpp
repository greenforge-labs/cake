// auto-generated DO NOT EDIT

#pragma once

#include <cstdint>
#include <string>

namespace cake {

/**
 * Convert a value to std::string for use in topic/service/action name construction.
 * These overloads support parameter substitution in names like "/robot/${param:id}/cmd_vel".
 */

inline std::string to_string(const std::string &value) { return value; }

inline std::string to_string(int value) { return std::to_string(value); }

inline std::string to_string(int64_t value) { return std::to_string(value); }

} // namespace cake
