#pragma once

namespace RailOS::Timetable {

/// Timetable entry types
enum class FormatType {
  NoFormat,
  TimeLoc,
  TimeTimeLoc,
  TimeCmd,
  StartNew,
  TimeCmdHeadCode,
  FinRemHere,
  FNSNonRepeatToShuttle,
  SNTShuttle,
  SNSShuttle,
  SNSNonRepeatFromShuttle,
  FSHNewService,
  Repeat,
  PassTime,
  ExitRailway
};

enum class LocationType {
  NoLocation,
  AtLocation,
  EnRoute,
  LocTypeForRepeatEntry
};

enum class SequenceType {
  NoSequence,
  Start,
  Finish,
  Intermediate,
  SequTypeForRepeatEntry
};

enum class ShuttleLinkType {
  NoShuttleLink,
  NotAShuttleLink,
  ShuttleLink,
  ShuttleLinkTypeForRepeatEntry
};

}; // namespace RailOS::Timetable