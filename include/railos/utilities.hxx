#pragma once

#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

#include "boost/date_time/posix_time/posix_time.hpp"

namespace RailOS::Utilities {
auto getDateTimeStamp() -> std::string;
auto getTimeStamp() -> std::string;
std::string format96HHMM(boost::posix_time::time_duration &date_time);
std::string format96HHMMSS(boost::posix_time::time_duration &date_time);
std::string incrementStringTimeOneMinute(const std::string &time_string);
}; // namespace RailOS::Utilities