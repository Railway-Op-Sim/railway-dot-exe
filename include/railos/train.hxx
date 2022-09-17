#pragma once

#include <memory>
#include <string>

#include "boost/date_time/posix_time/posix_time_config.hpp"
#include "railos/actions.hxx"
#include "railos/containers.hxx"
#include "railos/positioning.hxx"
#include "railos/logging.hxx"
#include "railos/utilities.hxx"

namespace RailOS::Train {
enum class Mode { NoMode, Timetable, Signaller };

class Train {
private:
//   friend TTrainController;
//   friend TInterface;

  // start data
  static const int call_on_max_speed{30};
  ///< km/h
  static const int max_mass_limit{10000000};
  ///< kg (i.e. 10,000 tonnes)
  static const int max_power_limit{100000000};
  ///< Watts (i.e. 100MW)
  static const int max_speed_limit = 400;
  ///< km/h

  static int next_train_id;
  ///< the ID value to be used for the next train that is created, static so
  ///< that it doesn't need an object to call it and its value is independent of
  ///< the objects

  std::string head_code;
  ///< needs own HeadCode because repeat entries will differ from
  ///< TrainDataEntry.HeadCode
  std::string follow_on_service_ref; // added at v2.12.0
  ///< used for terminating a service early and becoming new follow-on service
  bool skipped_departure{false};
  ///< used to indicate that a departure is still awaited when later timetabled
  ///< events are to be skipped
  bool action_skipped_flag{false};
  ///< prevents any further skipping until after the next departure
  bool hold_at_location_in_tt_mode{false};
  ///< true if actions are needed before train departs
  bool time_time_loc_arrived{false};
  ///< indicates whether has arrived (true) or not when
  ///< ActionVectorEntryPtr->FormatType == TimeTimeLoc
  bool remain_here_log_not_sent{false};
  ///< flag to prevent repeated logs, new at v1.2.0
  bool finish_join_log_sent{false}; // added at v2.4.0 to prevent repeatedly logging
                             // the event

  bool zero_power_no_front_split_msg{false}; // these added at v2.4.0
  bool zero_power_no_rear_split_msg{false};
  bool
      failed_train_no_finish_join_msg; // zero power trains can finish join, as
                                       // for empty stock, but not failed trains
  bool zero_power_no_joined_by_msg{false};
  bool zero_power_no_cdt_msg{false};
  bool zero_power_no_new_service_msg{false};
  bool zero_power_no_new_shuttle_from_non_repeat_msg{false};
  bool zero_power_no_repeat_shuttle_msg{false};
  bool ZeroPowerNoRepeatShuttleOrNewServiceMessage{false};
  ///< flags to indicate whether the respective message has been sent
  bool train_failure_pending{false};
  ///< set when failure due & takes effect when all PlotElements properly set,
  ///< added at v2.4.0

  int incremental_digits{0};
  ///< the number of digits to increment by in repeat entries
  int incremental_mins{0};
  ///< the number of minutes to increment by in repeat entries
  int rear_start_element{0};
  ///< start TrackVectorPosition element for rear of train
  int rear_start_exit_pos{0};
  ///< the LinkPos value for the rear starting element (i.e. links to the front
  ///< starting element)
  int repeat_number{0};
  ///< indicates which of the repeating services this train represents (0 =
  ///< first service)
  int signaller_max_speed{0};
  ///< maximum train speed under signaller control (in km/h)
  int start_speed{0};
  ///< the speed of the train when introduced into the railway (in km/h)
  int lead_element{-1}, lead_entry_pos{0}, lead_exit_pos{0}, mid_element{-1};
  int mid_entry_pos{0}, mid_exit_pos{0}, lag_element{-1}, leg_entry_pos{0}, lag_exit_pos{0};
  ///< TrackVector positions, & entry & exit connection positions for the
  ///< elements that the train occupies
  int train_id{0};
  ///< the train's identification number
  ///< TTrainDataEntry class)
  int skip_ptr_value{0};
  ///< stores the pointer increment from first action in ActionVector for
  ///< skipped actions when a departure is still awaited
  int train_skipped_events{0};
  ///< stores the pointer increment from the current action in ActionVector for
  ///< skipped actions when a departure is still awaited
  bool allowed_to_pass_red_signal{false};
  ///< set when train has been called on, or when under signaller control and
  ///< instructed to pass a red signal or to step forwards one element
  bool being_called_on{false};
  ///< in course of being called on to a station
  bool buffer_zoomout_flash_required{false};
  ///< set when train is at buffers and is to flash in zoomout mode (i.e. when
  ///< reaches buffers unexpectedly - more actions still being due in the
  ///< timetable)
  bool calling_on_flag{false};
  ///< calling on permitted
  bool departure_time_set{false};
  ///< set when stopped at a location and the next action is departure (set in
  ///< UpdateTrain when ReleaseTime and TRSTime have been established)
  bool first_half_move{false};
  ///< true when the train is on the first half of an element when it displays
  ///< as fully on two elements.  It only displays with the front character of
  ///< the headcode on the first half of an element when the halfway point of
  ///< the element has been passed.
  bool last_action_delay_flag{false};
  ///< used when trains join to ensure that there is a 30 second delay before
  ///< the actual join takes place after the two trains are adjacent to each
  ///< other
  bool last_sig_passed_failed{false};
  ///< flag used to erase route elements in an autosigs route after a failed
  ///< signal
  bool leaving_under_sig_controller_after_continuation{false};
  ///< set when the train has reached an exit continuation when under signaller
  ///< control, used to prevent the popup menu being given on right clicking
  ///< (can cause ambiguities in positioning if try to give signaller commands
  ///< when at or close to a continuation)
  bool one_length_accel_decel{false};
  ///< set when a train can only move forwards one element before stopping but
  ///< needs to accelerate for the first half of the element
  bool station_stop_calculated{false};
  ///< used in calculating DistanceToStationStop for trains running early before
  ///< they have reached the stop station
  bool step_forward_flag{false};
  ///< set when the signaller command to step forward one element has been given
  bool terminated_msg_sent{false};
  ///< set when a 'train terminated' message has been logged, to prevent its
  ///< being logged more than once
  bool train_failed{false}; // added at v2.4.0
  ///< indicates failure
  bool treat_pass_as_time_loc_departure{false}; // added at v2.12.0
  ///< only true when a train has become a follow-on service early and the
  ///< follow-on service normally passes the location, true treats it as a
  ///< departure
  double a_value;
  ///< this is a useful shorthand value in calculating speeds and transit times
  ///< in SetTrainMovementValues [= sqrt(2*PowerAtRail/Mass)]

  // the following values are used in calculating speeds and transit times in
  // SetTrainMovementValues, speeds in km/h & brake rates in m/sec/sec
  double entry_speed{0};
  ///< speed at which the train enters the next element
  double exit_speed_half{0};
  ///< speed when half way into the next element
  double exit_speed_full{0};
  ///< speed when leaving the next element
  double timetable_max_running_speed{0};
  ///< the maximum train running speed when in timetable mode (see int
  ///< SignallerMaxSpeed for signaller control)
  double max_exit_speed{0};
  ///< the maximum speed that the train can exit the next element
  double max_brake_rate{0};
  ///< the maximum brake rate that the train can achieve
  double brake_rate{0};
  ///< the current train brake rate
  double signaller_stop_brake_rate{0};
  ///< the train brake rate when stopping under signaller control
  double delayed_rand_mins{0};
  ///< the remaining random delay at any point in time for the train (added at
  ///< v2.13.0)
  double new_delay{0};
  ///< an additional random delay at a location (added at v2.13.0)
  double cumulative_delayed_rand_mins_one_train{0};
  ///< the running total of all random delays including knock-on delays for a
  ///< single train, used to reduce total late mins in performance summary
  ///< (added at v2.13.0)
  double power_at_rail{0};
  ///< in Watts (taken as 80% of the train's Gross Power, i.e. that entered by
  ///< the user)
  double original_power_at_rail{0};
  ///< new at v2.4.0 to store value before a failure so it can be restored from
  ///< here when repaired
  double mins_delayed{0.};
  ///< new at v2.2.0 for operator time to act panel. Calculated in UpdateTrain
  double op_time_to_act{60};
  ///< in minutes: new at v2.2.0 for operator time to act panel. Calculated in
  ///< UpdateTrain, -1 indicates not to be displayed on panel
  double time_to_exit{-1};
  ///< in minutes: new for multiplayer, -1 = > 60 mins
  double first_later_stop_recoverable_time{0.};
  ///< this used to deduct from RecoverableTime when arrive at a location for
  ///< OperatorActionpanel (OperatorActionPanel changed for ActionsDueForm at
  ///< v2.13.0)
  int front_element_speed_limit{0}, front_element_length{0};
  ///< values associated with the element immediately in front of the train
  ///< (speed in km/h, length in m)
  int mass{0};
  ///< in kg
  int distance_to_station_stop{0};
  ///< calculated in UpdateTrain & used in CalcDistanceToRedSignalandStopTime to
  ///< cater for trains running early before they have reached the stop station
  unsigned int update_counter{0};
  ///< used in train splitting operations to prevent too frequent checks for a
  ///< location being long enough for a split after a failure message has
  ///< already been given (doesn't need to stay failed as signaller can
  ///< manoeuvre it to a better location)
  boost::posix_time::time_duration entry_time{0, 0, 0, 0};
  boost::posix_time::time_duration exit_time_half{0, 0, 0, 0};
  boost::posix_time::time_duration exit_time_full{0, 0, 0, 0};
  ///< times used in SetTrainMovementValues corresponding to the next element
  ///< the train runs on
  boost::posix_time::time_duration release_time{0,0, 0, 0};
  boost::posix_time::time_duration trs_time{0, 0, 0, 0};
  boost::posix_time::time_duration actual_arrival_time{0, 0, 0, 0};
  ///< location departure time and 'train ready to start' time (TRSTime is 10
  ///< seconds before the ReleaseTime). ActualArrivalTime added at v2.13.0 for
  ///< random delays
  Containers::HVShortPair exit_pair{-1, -1};
  ///< H & V coordinates of the exit element related to TimeToExit, new for
  ///< multiplayer

  // operating data
  std::string restore_timetable_location;
  ///< stores the location name at which signaller control is taken, to ensure
  ///< that it is back at that location before timetable control can be restored
  bool plotted{false};
  ///< set when train plotted on screen
  bool train_gone{false};
  ///< set when train has left the railway, so it can be removed from the
  ///< display at the next clock tick
  bool spad_flag{false};
  ///< set when running past a red signal without permission flags to indicate
  ///< relevant stop conditions or pending stop conditions

  int h_offset[4]{0, 0, 0, 0}, v_offset[4]{0, 0, 0, 0};
  ///< each headcode character is an 8x8 pixel graphic and must be placed within
  ///< a 16x16 pixel element, these values set the horizontal & vertical offsets
  ///< of the top left hand corner character graphic relative to the 16x16
  ///< element
  int old_zoom_out_element[3]{-1, -1, -1};
  ///< stores the Lead, Mid & Lag TrackVectorPositions, used for unplotting
  ///< trains from the old position in zoomed-out mode prior to replotting in a
  ///< new position (new will be different from old if train moving)
  int plot_element[4]{-1, -1, -1};
  ///< the TrackVectorPosition of the element where each of the 4 headcode
  ///< characters is plotted  (need to be set to Lead, Mid & Lag elemnt values
  ///< before PlotGraphic called)
  int plot_entry_pos[4]{0, 0, 0};
  ///< the LinkPos value corresponding to the train entry link of the element
  ///< where each of the 4 headcode characters is plotted
  int train_crashed_into{0};
  ///< the TrainID of the train that this train has crashed into, recorded so
  ///< that train can be marked and displayed as crashed also

  // TODO: Add in Graphics members ==============================

  //     Graphics::TBitmap *HeadCodePosition[4];
  // ///< Set from the HeadCodeGrPtr[4] pointer values, HeadCodePosition[0] is
  // always the front, which may be the first or the last headcode character
  // depending on the direction of motion and the entry link value of the
  // element being entered (see also LowEntryValue(int EntryLink) above)
  //     Graphics::TBitmap *BackgroundPtr[4];
  // ///< the existing track graphic that the train headcode segment covers up
  // (one for each headcode segment)
  //     Graphics::TBitmap *FrontCodePtr;
  // ///< points to the front headcode segment, this is set to red or blue
  // depending on TrainMode
  //     Graphics::TBitmap *HeadCodeGrPtr[4];
  // ///< points to the headcode segment graphics e.g. 5,A,4,7.

  //     TColor BackgroundColour;

  //=============================================================

  ///< the background colour of the train's headcode graphics
  Positioning::Straddle straddle;
  ///< the current Straddle value of the train (see TStraddle above)
  std::string sel_skip_string;
  ///< the selected timetable string when skipping timetabled events

  public:

  int getID() const {return train_id;}

  bool hasCrashed() const {return crashed;}

  bool hasDerailed() const {return derailed;}

  int getLagElement() const {return lag_element;}

  int getRearStartElement() const {return rear_start_element;}

  int getLeadEntryPosition() const {return lead_entry_pos;}

  int getMidEntryPosition() const {return mid_entry_pos;}

  int getMidElement() const {return mid_element;}

  int getLeadElement() const {return lead_element;}

  int getSignallerMaxSpeed() const {return signaller_max_speed;}

  int getCumulativeDelayedRandMinsOneTrain() const {return cumulative_delayed_rand_mins_one_train;}

  int getRepeatNumber() const {return repeat_number;}

  std::string getHeadCode() const {return head_code;}

  std::shared_ptr<Actions::TrainDataEntry> train_data_entry;
  ///< points to the current position in the timetable's TrainDataVector
  std::shared_ptr<Actions::VectorEntry> action_vector_entry;
  ///< points to the current position in the ActionVector (a member of the
  bool signaller_removed{false};
  ///< set when removed under signaller control to force a removal from the
  ///< display at the next clock tick
  boost::posix_time::time_duration last_action_time{0, 0, 0, 0};
  ///< time of the last timetabled event, used to ensure at least a 30 second
  ///< delay before the next action
  double max_running_speed{0};
  ///< the current maximum train running speed
  bool timetable_finished{false};
  ///< set when there are no more timetable actions
  bool signaller_stopping_flag{false};
  ///< set when the signaller stop command has been given
  Mode train_mode{Mode::NoMode};
  ///< mode of operation - either Timetable (running under timetable control) or
  ///< Signaller (running under signaller control)

  bool derailed{false}, derail_pending{false}, crashed{false}, stopped_at_buffers{false};
  bool stopped_at_signal{false}, stopped_at_location{false}, signaller_stopped{false};
  bool stopped_after_spad{false}, stopped_for_train_in_front{false};
  bool stopped_without_power{false}, not_in_service{false};
  bool joined_other_train_flag{false};
  ///< true when the train has joined another train following an 'Fjo' timetable
  ///< command or a signaller join (when set the train is removed from the
  ///< display at the next clock tick)

  // inline functions
  bool revisedStoppedAtLoc() const // added at v2.12.0
  {
    return (stopped_at_location || treat_pass_as_time_loc_departure);
    ///< This function is used in place of StoppedAtLocation in most places
    ///< where it appeared so new service passes are treated as departures when
    ///< terminationg trains early and becoming the follow-on service
  }

  // functions defined in .cxx file

  /// called during doubleingLabelNextString to find the next service departure
  /// time & next location
  std::string getNewServiceDepartureInfo(
      int caller, std::unique_ptr<Actions::VectorEntry> ptr, int repeat_number,
      std::unique_ptr<Actions::TrainDataEntry> linked_train_data,
      const std::string &ret_str) const;
  /// Used in the doubleing window to display the 'Next' action
  std::string
  doubleingLabelNextString(int caller,
                          std::unique_ptr<Actions::VectorEntry> ptr) const;
  /// Used in the doubleing window to display the timetable
  std::string
  doubleingTimetableString(int caller,
                          std::unique_ptr<Actions::VectorEntry> ptr) const;
  /// Returns the train headcode, taking account of the RepeatNumber
  std::string getTrainHeadCode(int caller) const;
  /// Carries out an integrity check for the train section of a session file, if
  /// fails a message is given and the file is not loaded.  It is static so that
  /// it can be called without needing an object.
  static bool checkOneSessionTrain(std::ifstream &in_Stream);
  ////Used to check for an adjacent train in signaller mode for use in PopUp
  ///menu //new at v2.4.0
  bool hasAdjacentTrain(int caller,
                        std::unique_ptr<Train> &TrainToBeJoinedBy) const;
  /// Indicates that a train is not prevented from moving - used to allow
  /// appropriate popup menu options when under signaller control
  bool canMove(int caller) const;
  /// Indicates that a train is only prevented from moving by a signal - used to
  /// allow appropriate popup menu options when under signaller control
  bool canMoveBySignalRelease(int caller) const;
  /// Checks forward from train LeadElement, following leading point attributes
  /// but ignoring trailing point attributes, until finds either a train or a
  /// signal/buffers/continuation/loop.  If finds a train returns false, else
  /// returns true.  Used when train stopped for a train in front, to ensure it
  /// remains stopped until this function returns true.
  bool clearToNextSignal(int caller) const;
  /// True when the train is stopped at a timetabled location
  bool atTimetableLocation(int caller, std::string &location_name) const;
  /// True if Element is a buffer and Exitpos is the buffer end
  bool bufferAtExit(int caller, int element, int exit_pos) const;
  /// True if the train can be called on at its current position - see detail in
  /// .cxx file
  bool callingOnAllowed(int caller) const;
  /// True if Element is a continuation and Exitpos is the continuation end
  bool continuationExit(int caller, int element, int exit_pos) const;
  /// True if train is on a bridge on trackpos 0 & 1
  bool isTrainIDOnBridgeTrackPos01(int caller,
                                   unsigned int track_vector_position) const;
  /// True if train is on a bridge on trackpos 2 & 3
  bool isTrainIDOnBridgeTrackPos23(int caller,
                                   unsigned int track_vector_position) const;
  /// True if train service terminates at its current location
  bool isTrainTerminating(int caller) const;
  /// Returns true if EntryLink is 1, 2, 4 or 7, in these circumstances the
  /// front of the train (i.e. the character that is red or blue) is the last
  /// character of the headcode, otherwise it's the first character of the
  /// headcode
  bool lowEntryValue(int entry_link) const;
  /// Returns true if any part of train on a continuation - called when checking
  /// for failures, prevent if on a continuation
  bool trainOnContinuation(int caller) const; // new at v2.4.0
  /// True for a train waiting to be joined when the joining train is adjacent
  bool trainToBeJoinedByIsAdjacent(int caller,
                                   std::unique_ptr<Train> &other_train) const;
  /// True for a train waiting to join another when the other train is adjacent
  bool trainToJoinIsAdjacent(int caller,
                             std::unique_ptr<Train> &other_train) const;

  /// Returns the number by which the train ActionVectorEntryPtr needs to be
  /// incremented to point to the location arrival entry or passtime entry
  /// before a change of direction.
  /** Used to display missed actions when a stop or pass location has been
  reached before other timetabled events have been carried out. If can't find
  it, or Name is "", -1 is returned.  A change of direction is the limit of the
  search because a train may not stop at a location on the way out but stop on
  way back, and in these circumstances no actions have been missed.  Stop
  indicates whether the train will stop at (true) or pass (false) the
  location.*/
  int nameInTimetableBeforeCDT(int caller, const std::string &name,
                               bool stop) const;

  /// new v2.2.0 for operator action panel.  Calculates the time left for
  /// operator action to avoid unnecessary delays.  Added TimeToExit & ExitPair
  /// for multiplayer
  double calcTimeToAct(int caller, double time_to_exit,
                      Containers::HVShortPair &exit_pair);

  // TODO: Fix graphics members
  /// Return a pointer to the graphic corresponding to the character 'CodeVhar'
  // Graphics::TBitmap *SetOneGraphicCode(char CodeChar);

  /// Returns the timetable action time corresponding to 'Time' for this train,
  /// i.e. it adjusts the time value according to the train's RepeatNumber and
  /// the incremental minutes between repeats
  boost::posix_time::time_duration
  getTrainTime(int caller, const boost::posix_time::time_duration &time) const;

  /// Reverses the direction of motion of the train
  void changeTrainDirection(int caller, bool no_log)
      const; // NoLogFlag added at v2.12.0 for new service TT skips
  /// This is a housekeeping function to delete train heap objects (bitmaps)
  /// explicitly rather than by using a destructor.
  /**This is because vectors erase elements during internal operations & if
  TTrain had an explicit destructor that deleted the heap elements then it would
  be called when a vector element was erased. Calling the default TTrain
  destructor doesn't matter because all that does is release the memory of the
  members (including pointers to the bitmaps), it doesn't destroy the bitmaps
  themselves. It's important therefore to call this function before erasing the
  vector element, otherwise the pointers to the bitmaps would be lost and the
  bitmaps never destroyed, thereby causing memory leaks.*/
  void deleteTrain(int caller);
  /// Checks whether Element and EntryPos (where train is about to enter) is on
  /// an existing route (or crosses or meets an existing route for crossings and
  /// points) that isn't the train's own route and cancels it if so - because it
  /// has reached it at the wrong point
  void checkAndCancelRouteForWrongEndEntry(int caller, int element,
                                           int entry_pos);
  /// Carry out the actions needed when a train is waiting to join another train
  void finishJoin(int caller);
  /// Carry out the actions needed when a train is to split from the front
  void frontTrainSplit(int caller);
  /// Called when a train is about to leave an element and move onto another.
  /**This function obtains the new element that will become the train's
  LeadElement, the earlier LeadElement having been assigned to MidElement and
  earlier MidElement
  /// assigned to LagElement.  It assumes Mid & Lag already set, sets
  LeadElement, LeadEntryPos, LeadExitPos & DerailPending (i.e. about
  /// to foul points but train only becomes derailed when it moves fully onto
  the element)*/
  void getLeadElement(int caller);
  /// Sets HOffset & VOffset (see above) for a single headcode character
  /// depending on the Link value
  void getOffsetValues(int caller, int &h_offset, int &v_offset,
                       int link) const;
  /// Carry out the actions needed when a train is waiting to be joined by
  /// another train
  void joinedBy(int caller);
  /// Create one train with relevant member values from the sesion file
  void loadOneSessionTrain(int caller, std::ifstream &in_stream);
  /// Send a message to the performance log and performance file, and if the
  /// message is flagged as a warning then it is also sent as a warning (in red
  /// at the top of the railway display area)
  void logAction(int caller, const std::string &head_code,
                 std::string &other_headcode, Actions::Type action_type,
                 std::string &location_name,
                 boost::posix_time::time_duration &timetable_non_repeat_time,
                 bool warning);
  /// Carry out the actions needed when a new shuttle service is created from a
  /// non-repeating (F-nshs) service
  void newShuttleFromNonRepeatService(
      int caller, bool no_log); // bool NoLogFlag added at v2.12.0 for new
                                   // service TT skips
  /// Carry out the actions needed when a train forms a new service (code Fns)
  void newTrainService(int caller,
                       bool no_log); // bool NoLogFlag added at v2.12.0 for
                                        // new service TT skips
  /// Store the background bitmap pointer (BackgroundPtr - see above) prior to
  /// being overwritten by the train's headcode character, so that it can be
  /// replotted after the train has passed using PlotBackgroundGraphic.  Note
  /// that this doesn't pick up the actual graphic, it reconstructs the track
  /// graphic with autosigs route if set, so any text or user graphics at that
  /// position will be blanked out by the train until the next
  /// ClearandRebuildRailway
// TODO: Add graphics function
//   void pickUpBackgroundBitmap(int caller, int h_offset, int v_offset, int element,
//                               int entry_pos,
//                               Graphics::TBitmap *GraphicPtr) const;
  /// When a train moves off a bridge the other track may contain a route or
  /// have a train on it that has been obscured by this train.  This function
  /// checks and replots the original graphic if necessary
  void plotAlternativeTrackRouteGraphic(int caller, unsigned int lag_element,
                                        int lag_elink_pos, int h_offset,
                                        int v_offset, Positioning::Straddle straddle);
  /// Replot the graphic pointed to by BackgroundPtr (see above) after a train
  /// has passed
  //TODO: Graphics
//   void plotBackgroundGraphic(int caller, int array_number, TDisplay *Disp) const;
  /// Plots the train and sets up all relevant members for a new train when it
  /// is introduced into the railway
  void plotStartPosition(int caller) const;
  /// Plots the train on the display in normal (zoomed-in) mode
//   void plotTrain(int Caller, TDisplay *Disp);
  /// Plot the train's headcode character corresponding to ArrayNumber
 // void plotTrainGraphic(int Caller, int ArrayNumber, TDisplay *Disp);
  /// Plots the train on screen in zoomed-out mode, state of 'Flash' determines
  /// whether the flashing trains are plotted or not
  void plotTrainInZoomOutMode(int caller, bool flash);
  /// Changes the train's background colour (e.g. to pale green if stopped at a
  /// station) Note that this uses the PlotElement[4] values, so whenever called
  /// these should reflect the Lead, Mid and Lag Element values or will be
  /// plotted in the wrong position
//   void plotTrainWithNewBackgroundColour(int caller, TColor NewBackgroundColour,
//                                         TDisplay *Disp);
  /// Carry out the actions needed when a train is to split from the rear
  void rearTrainSplit(int caller);
  /// Sends the 'train terminated' message to the performance log and sets
  /// TimetableFinished to true
  void remainHere(int caller);
  /// Carry out the actions needed to create either a new shuttle service or (if
  /// all repeats have finished) a non-repeating shuttle finishing service (code
  /// is Fns-sh)
  void repeatShuttleOrNewNonRepeatService(
      int caller, bool no_log); // bool NoLogFlag added at v2.12.0 for new
                                   // service TT skips
  /// Carry out the actions needed to create either a new shuttle service or (if
  /// all repeats have finished) to keep train at its current location (code is
  /// Frh-sh)
  void
  repeatShuttleOrRemainHere(int caller,
                            bool no_log); // bool NoLogFlag added at v2.12.0
                                             // for new service TT skips
  /// After a train has moved off an element that element has its
  /// TrainIDOnElement value set back to -1 to indicate that a train is not
  /// present on it, but, if the element is a bridge then the action is more
  /// complex because the element's TrainIDOnBridgeTrackPos01 &/or
  /// TrainIDOnBridgeTrackPos23 values are involved
  void resetTrainElementID(int caller, unsigned int track_vector_pos,
                           int entry_pos);
  /// Data for a single train is saved to a session file
  void saveOneSessionTrain(int caller, std::ofstream &out_stream);
  /// Missed actions (see NameInTimetableBeforeCDT above) sent to the
  /// performance log and performance file
  void sendMissedActionLogs(int caller, int inc_num, const std::shared_ptr<Actions::VectorEntry> ptr);
  /// Set the four HeadCodeGrPtr[4] pointers to the appropriate character
  /// graphics with the current backgroundcolour by calling SetOneGraphicCode
  /// four times.  This doesn't plot anything on screen, it just sets the
  /// graphics which can be plotted at the appropriate position when required
  void setHeadCodeGraphics(int caller, std::string &code);
  /// When a train moves onto an element that element has its TrainIDOnElement
  /// value set to the TrainID value to indicate that a train is present on it.
  /// If the element is a bridge then the element's TrainIDOnBridgeTrackPos01 or
  /// TrainIDOnBridgeTrackPos23 value is also set as appropriate. Also if the
  /// element is a flashing gap (LeadElement used as that lands on it first)
  /// then the flashing stops
  void setTrainElementID(int caller, unsigned int track_vector_pos,
                         int entry_pos);
  /// Calculates train speeds and times for the element that the train is about
  /// to enter.  See flowchart
  /** \htmlinclude SetTrainMovementValues.html */
  void setTrainMovementValues(int caller, int track_vector_pos,
                              int entry_pos);
  /// Unplots & replots train, which checks for facing signal and sets
  /// StoppedAtSignal if req'd
  void signallerChangeTrainDirection(int caller);
  /// Called when there is a random train failure
  void trainHasFailed(int caller);
  /// Unplot train from screen in zoomed-in mode
  void unplotTrain(int caller);
  /// Unplot train from screen in zoomed-out mode
  void unplotTrainInZoomOutMode(int caller);
  /// Major function called at each clock tick for each train & handles all
  /// train movement & associated actions.  See flowchart
  /** \htmlinclude UpdateTrain.html */
  void updateTrain(int caller);
  /// Called by TTrainController::WriteTrainsToImage (called by
  /// TInterface::SaveOperatingImage1Click) to add all a single train graphic to
  /// the image file
//   void writeTrainToImage(int Caller, Graphics::TBitmap *Bitmap);

  // inline functions

  /// Check whether the train has left the railway, so that it can be removed
  /// from the display at the next clock tick
  bool hasTrainGone() { return train_gone; }

public:
  // inline functions

  /// True if the train has stopped for any reason
  bool Stopped() {
    return (
        crashed || derailed || stopped_at_buffers || stopped_at_signal ||
        stopped_at_location || signaller_stopped || stopped_after_spad ||
        stopped_for_train_in_front || stopped_without_power || not_in_service ||
        treat_pass_as_time_loc_departure); // added ' || TreatPassAsTimeLocDeparture'
                                      // at v2.12.0
  }
  /// get LeadElement - used in RouteLockingRequired in TrackUnit.cpp
  int getLeadElement() { return lead_element; }
  /// Added at v1.2.0: true if any part of train on specific link, false
  /// otherwise, including no link present & no TrackVectorNumber within Lead,
  /// Mid or Lag (public so Track->TrainOnLink can access it)

  /// functions in .cxx file
  bool linkOccupied(int caller, int track_vector_pos, int link_number);
  /// Constructor, sets listed member values
  Train(int caller, int rear_start_elem_in, int rear_start_exit_pos_in,
         const std::string &input_code, int start_speed, int mass, double max_running_speed,
         double max_brake_rate, double power_at_rail, Mode train_mode,
         std::shared_ptr<Actions::TrainDataEntry> &train_data_entry, int repeat_number,
         int incremental_mins, int incremental_digits, int signaller_max_speed);
};

}; // namespace RailOS::Train