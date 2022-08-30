#pragma once

#include <string>

namespace RailOS::Positioning {
class ExitInfo // corresponds to TServiceInfo in Interface  12 bytes
{
public:
  std::string service_reference = std::string(8, ' ');
  int n_repeats{0};
  int n_secs_to_exit{-1}; //-1 = >= 60 mins

  ExitInfo(); // default constructor to initialise values to 8 spaces or 0
};

/// Defines the train position with respect to the track elements; three consecutive elements are Lead (front), Mid (middle) and Lag (rear), and a train is either fully on two of the elements (LeadMid or MidLag), or is straddling all three elements (LeadMidLag).  As the train moves forwards the element that was Lead becomes Mid and Mid becomes Lag.
enum class Straddle
{
    MidLag, LeadMidLag, LeadMid
};
}; // namespace RailOS::Positioning