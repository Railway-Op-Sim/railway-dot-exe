#pragma once

#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>

#include "boost/date_time/posix_time/posix_time.hpp"

namespace RailOS::Utilities {
extern int cumulative_delayed_rand_mins_all_trains;
extern bool clock_2_stopped;
auto getDateTimeStamp() -> std::string;
auto getTimeStamp() -> std::string;
std::string format96HHMM(boost::posix_time::time_duration &date_time);
std::string format96HHMMSS(boost::posix_time::time_duration &date_time);
std::string incrementStringTimeOneMinute(const std::string &time_string);

extern int default_track_length;
extern int default_track_speed_limit;
///< length of each track element before being changed within the program (can be changed in config.txt) (moved here at v2.13.1 so doesn't change)

}; // namespace RailOS::Utilities