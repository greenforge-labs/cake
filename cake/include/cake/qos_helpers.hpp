// auto-generated DO NOT EDIT

#pragma once

#include <rmw/types.h>
#include <stdexcept>
#include <string>

namespace cake {

/**
 * Convert a string parameter value to RMW reliability policy.
 * @param value The string value ("RELIABLE" or "BEST_EFFORT")
 * @return The corresponding rmw_qos_reliability_policy_t
 * @throws std::runtime_error if the value is invalid
 */
inline rmw_qos_reliability_policy_t to_reliability(const std::string &value) {
    if (value == "RELIABLE") {
        return RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    } else if (value == "BEST_EFFORT") {
        return RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    } else {
        throw std::runtime_error("Invalid reliability policy: '" + value + "'. Expected 'RELIABLE' or 'BEST_EFFORT'.");
    }
}

/**
 * Convert a string parameter value to RMW durability policy.
 * @param value The string value ("TRANSIENT_LOCAL" or "VOLATILE")
 * @return The corresponding rmw_qos_durability_policy_t
 * @throws std::runtime_error if the value is invalid
 */
inline rmw_qos_durability_policy_t to_durability(const std::string &value) {
    if (value == "TRANSIENT_LOCAL") {
        return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    } else if (value == "VOLATILE") {
        return RMW_QOS_POLICY_DURABILITY_VOLATILE;
    } else {
        throw std::runtime_error(
            "Invalid durability policy: '" + value + "'. Expected 'TRANSIENT_LOCAL' or 'VOLATILE'."
        );
    }
}

/**
 * Convert a string parameter value to RMW liveliness policy.
 * @param value The string value ("AUTOMATIC" or "MANUAL_BY_TOPIC")
 * @return The corresponding rmw_qos_liveliness_policy_t
 * @throws std::runtime_error if the value is invalid
 */
inline rmw_qos_liveliness_policy_t to_liveliness(const std::string &value) {
    if (value == "AUTOMATIC") {
        return RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    } else if (value == "MANUAL_BY_TOPIC") {
        return RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    } else {
        throw std::runtime_error(
            "Invalid liveliness policy: '" + value + "'. Expected 'AUTOMATIC' or 'MANUAL_BY_TOPIC'."
        );
    }
}

} // namespace cake
