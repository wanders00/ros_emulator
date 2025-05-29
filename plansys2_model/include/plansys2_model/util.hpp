#pragma once

#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

// Fills the emulated_response fields from environment variables with the given prefix (e.g., "ROBOT" or "GANTRY")
template <typename EmulatedResponseT>
void fill_emulated_response_from_env(const std::string& prefix, EmulatedResponseT& response) {
    auto get_env_or_zero = [](const char* var) -> int {
        const char* val = std::getenv(var);
        if (val) {
            try {
                return std::stoi(val);
            } catch (...) {
                return 0;
            }
        }
        return 0;
    };
    response.emulate_execution_time = get_env_or_zero((prefix + "_EMULATE_EXECUTION_TIME").c_str());
    response.emulated_execution_time = get_env_or_zero((prefix + "_EMULATED_EXECUTION_TIME").c_str());
    response.emulate_failure_rate = get_env_or_zero((prefix + "_EMULATE_FAILURE_RATE").c_str());
    response.emulated_failure_rate = get_env_or_zero((prefix + "_EMULATED_FAILURE_RATE").c_str());
    response.emulate_failure_cause = get_env_or_zero((prefix + "_EMULATE_FAILURE_CAUSE").c_str());
    const char* causes_env = std::getenv((prefix + "_EMULATED_FAILURE_CAUSE").c_str());
    if (causes_env) {
        std::stringstream ss(causes_env);
        std::string cause;
        response.emulated_failure_cause.clear();
        while (std::getline(ss, cause, ',')) {
            response.emulated_failure_cause.push_back(cause);
        }
    }
}
