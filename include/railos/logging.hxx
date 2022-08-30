#pragma once

#include "spdlog/sinks/basic_file_sink.h"
#include <deque>
#include <stdexcept>

namespace RailOS::Logging {
    extern std::deque<std::string> call_log;
    extern void popCallLog(int caller) {
        if(call_log.empty()) {
            throw std::runtime_error("CallLog pop call when empty! Caller = " + std::to_string(caller));
        }
        call_log.pop_back();
    }
};