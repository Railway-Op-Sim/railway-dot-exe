#include "railos/utilities.hxx"
#include "boost/date_time/posix_time/posix_time_config.hpp"
#include "boost/date_time/time_duration.hpp"
#include <iomanip>
#include <sstream>

namespace RailOS::Utilities {
  int cumulative_delayed_rand_mins_all_trains{0};
  bool clock_2_stopped{false};
auto getDateTimeStamp() -> std::string {
  auto null_time = std::time(nullptr);
  auto tm_now = *std::localtime(&null_time);
  std::stringstream str_stream;

  str_stream << std::put_time(&tm_now, "%d/%m/%Y %H:%M:%S");

  return str_stream.str();
}

auto getTimeStamp() -> std::string {
  auto null_time = std::time(nullptr);
  auto tm_now = *std::localtime(&null_time);
  std::stringstream str_stream;

  str_stream << std::put_time(&tm_now, "%H:%M:%S");

  return str_stream.str();
}

std::string format96HHMMSS(boost::posix_time::time_duration &date_time)
// Formats a boost time_duration into a std::string of the form hh:mm:ss where
// hh runs from 00 to 95 & resets when it reaches 96
{
  std::stringstream str_stream;
  boost::posix_time::time_facet *time_fac =
      new boost::posix_time::time_facet(":%M:%S");
  str_stream.imbue(std::locale(std::locale::classic(), time_fac));
  str_stream << date_time;

  int hours_ = date_time.hours();

  while (hours_ >= 96) {
    hours_ -= 96;
  }

  std::string hour_str_ = std::to_string(hours_);

  while (hour_str_.size() < 2) {
    hour_str_ = "0" + hour_str_;
  }

  return hour_str_ + str_stream.str();
}

std::string format96HHMM(boost::posix_time::time_duration &date_time)
// Formats a boost time_duration into a std::string of the form hh:mm where hh
// runs from 00 to 95 & resets when it reaches 96
{
  std::stringstream str_stream;
  boost::posix_time::time_facet *time_fac =
      new boost::posix_time::time_facet(":%M");
  str_stream.imbue(std::locale(std::locale::classic(), time_fac));
  str_stream << date_time;

  int hours_ = date_time.hours();

  while (hours_ >= 96) {
    hours_ -= 96;
  }

  std::string hour_str_ = std::to_string(hours_);

  while (hour_str_.size() < 2) {
    hour_str_ = "0" + hour_str_;
  }

  return hour_str_ + str_stream.str();
}

std::string incrementStringTimeOneMinute(const std::string &time_string)
{
    boost::posix_time::time_duration duration_(0, 0, 0);
    std::stringstream str_stream_;
    str_stream_ << time_string;
    str_stream_ >> duration_;
    duration_ += boost::posix_time::time_duration(0, 1, 0);
    str_stream_.clear();
    boost::posix_time::time_facet *time_fac =
      new boost::posix_time::time_facet("%H:%M");
    str_stream_.imbue(std::locale(std::locale::classic(), time_fac));
    str_stream_ << duration_;
    return str_stream_.str();
}
}; // namespace RailOS::Utilities