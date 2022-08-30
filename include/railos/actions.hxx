#pragma once

#include <string>
#include <vector>

#include "boost/date_time/posix_time/posix_time.hpp"
#include "boost/date_time/posix_time/posix_time_config.hpp"

#include "railos/containers.hxx"
#include "railos/timetable.hxx"

namespace RailOS::Actions {

enum class EventType {
  NoEvent,
  FailTrainEntry,
  FailCreateTrain,
  FailCreateOnRoute,
  FailCreatePoints,
  FailSPAD,
  FailLockedRoute,
  FailLocTooShort,
  FailSplitDueToOtherTrain,
  FailCrashed,
  FailDerailed,
  FailUnexpectedBuffers,
  FailUnexpectedExitRailway,
  FailMissedArrival,
  FailMissedSplit,
  FailMissedJBO,
  FailMissedJoinOther,
  FailMissedTerminate,
  FailMissedNewService,
  FailMissedExitRailway,
  FailMissedChangeDirection,
  FailMissedPass,
  FailCreateLockedRoute,
  FailEnterLockedRoute,
  WaitingForJBO,
  WaitingForFJO,
  FailBuffersPreventingStart,
  FailBufferCrash,
  FailLevelCrossingCrash,
  FailIncorrectExit,
  ShuttleFinishedRemainingHere,
  RouteForceCancelled,
  FailEntryRouteSetAgainst
};

enum class Type {
  Arrive,
  Terminate,
  Depart,
  Create,
  Enter,
  Leave,
  FrontSplit,
  RearSplit,
  JoinedByOther,
  ChangeDirection,
  NewService,
  TakeSignallerControl,
  RestoreTimetableControl,
  RemoveTrain,
  SignallerMoveForwards,
  SignallerJoin,
  TrainFailure, // SignallerJoin, TrainFailure & RepairFailedTrain new at v2.4.0
  RepairFailedTrain,
  SignallerChangeDirection,
  SignallerPassRedSignal,
  Pass,
  SignallerControlStop,
  SignallerStop,
  SignallerLeave,
  SignallerStepForward
};

class TrainDataEntry;

/// Contains a single train action in a timetable - repeat entry is also of this class though no train action is taken for it
class VectorEntry
{
public:
    std::string location_name, command, other_headcode, non_repeating_shuttle_link_headcode;
///< string values for timetabled event entries, null on creation
//Other HeadCode & NonRepeatingShuttleLinkHeadCode have service ref entered in ProcessOneTimetableLine but these are
//changed back to basic HeadCodes as almost the final action in SecondPassActions (uses StripExcessFromHeadCode)
    bool under_signaller_control = false;

///< indicates a train that is defined by the timetable as under signaller control
    bool trigger_warning_panel_alert = false;

///< if set triggers an alert in the warning panel when the action is reached
    int n_repeats{0};

///< the number of repeating services
    int rear_start_or_repeat_mins{0};
    int front_start_or_repeat_digits{0};

///< dual-purpose variables used for the TrackVectorPositions of the rear and front train starting elements (for Snt) or for repeat minute & digit values in repeat entries
    boost::posix_time::time_duration event_time{0, 0, 0, 0};
    boost::posix_time::time_duration arrival_time{0, 0, 0, 0};
    boost::posix_time::time_duration departure_time{0, 0, 0, 0};

///< relevant times at which the action is timetabled, zeroed on creation so change to -1 as a marker for 'not set'
    Containers::NumList exit_list;

///< the list of valid train exit TrackVector positions for 'Fer' entries (empty to begin with)
    Timetable::FormatType format_type = Timetable::FormatType::NoFormat;
///< defines the timetable action type
    Timetable::LocationType location_type = Timetable::LocationType::NoLocation;
///< indicates where the train is when the relevant action occurs
    Timetable::SequenceType sequence_type = Timetable::SequenceType::NoSequence;
///< indicates where in the sequence of codes the action lies
    Timetable::ShuttleLinkType shuttle_link_type = Timetable::ShuttleLinkType::NoShuttleLink;
///< indicates whether or not the action relates to a shuttle service link
    std::unique_ptr<TrainDataEntry> linked_train_entry = nullptr;
///< link pointer for use between fsp/rsp & Sfs; Fjo & jbo; Fns & Sns; & all shuttle to shuttle links
    std::unique_ptr<TrainDataEntry> non_repeating_shuttle_link_entry = nullptr;
///< pointer used by shuttles for the non-shuttle train links, in & out, the corresponding non-shuttle linked trains use LinkedTrainEntryPtr

};

typedef std::vector<VectorEntry>Vector;
///< contains all actions for a single train
typedef Vector::iterator VectorIterator;
///< iterator

/// contains status info for each train
enum RunningEntry
{
    NotStarted, Running, Exited
};

class TrainOperatingData
{
public:
    int train_id = -1;
    Actions::EventType event_reported = Actions::EventType::NoEvent;
    RunningEntry running_entry = RunningEntry::NotStarted;
};

typedef std::vector<TrainOperatingData>TrainOperatingDataVector;
///< vector containing operational data for each timetabled train including all repeats

/// Contains all data for a single timetable service entry
class TrainDataEntry
{
public:
    std::string head_code, service_reference, description;
///< headcode is the first train's headcode, rest are calculated from repeat information; ServiceReference is the full (up to 8 characters) reference from the timetable (added at V0.6b)
    double max_brake_rate;
///< in metres/sec/sec
    double max_running_speed{0};
///< in km/h
    double power_at_rail;
///< in Watts (taken as 80% of the train's Gross Power, i.e. that entered by the user)
    int mass;
///< in kg
    int n_trains{0};
///< number of repeats + 1
    int signaller_speed;
///< in km/h for use when under signaller control
    int start_speed{0};
///< in km/h
    Actions::Vector actions;
///< all the actions for the train
    TrainOperatingDataVector train_operating_data;
///< operating information for the train including all its repeats
};

/// A single train timetable action for use in a formatted timetable
class OneTrainFormattedEntry
{
public:
    std::string action;
///< includes location if relevant
    std::string time;
///< the time of the action as a string
};

typedef std::vector<OneTrainFormattedEntry>OneFormattedTrainVector;
///< vector of formatted timetable actions for a single train

/// A single train with its headcode + list of actions for use in the formatted timetable
class OneCompleteFormattedTrain
{
public:
    std::string head_code;
    OneFormattedTrainVector trains;
};

typedef std::vector<OneCompleteFormattedTrain>OneCompleteFormattedTrainVector;
///< vector of a timetabled train with all its repeats for use in the formatted timetable

// ---------------------------------------------------------------------------

/// Contains all information for a single timetable entry for use in the formatted timetable
class TrainFormattedInformation
{
public:
    std::string Header;
///< description, mass, power, brake rate etc
    int NumberOfTrains;
///< number of repeats + 1
    OneCompleteFormattedTrainVector trains;
};

typedef std::vector<TrainFormattedInformation>AllFormattedTrains;
///< vector of all timetabled trains for use in the formatted timetable

}; // namespace RailOS::Actions