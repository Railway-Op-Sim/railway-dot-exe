#include "railos/train_controller.hxx"
#include "boost/date_time/posix_time/posix_time_config.hpp"
#include "railos/actions.hxx"
#include "railos/globals.hxx"
#include "railos/logging.hxx"
#include "railos/timetable.hxx"
#include "railos/track.hxx"
#include "railos/utilities.hxx"
#include <memory>
#include <stdexcept>
#include <string>

namespace RailOS::Train {
TrainController::~TrainController() {
  for (unsigned int x = 0; x < trains.size(); x++) {
    trainVectorAt(32, x).deleteTrain(4);
  }
  trains.clear();
}

void TrainController::logEvent(std::string str) {
  std::stringstream str_stream;
  boost::posix_time::time_facet *time_fac =
      new boost::posix_time::time_facet("%H:%M:%S");
  str_stream.imbue(std::locale(std::locale::classic(), time_fac));
  str_stream << ttb_clock_time;
  std::string full_str_ =
      Utilities::getTimeStamp() + "," + str_stream.str() + "," + str;

  // restrict to last 1000 entries
  Logging::event_log.push_back(full_str_);
  if (Logging::event_log.size() > 1000) {
    Logging::event_log.pop_front();
  }
}

void TrainController::operate(int caller) {
  Logging::call_log.push_back(Utilities::getTimeStamp() + "," +
                              std::to_string(caller) + ",Operate");

  const bool clock_state_ = Globals::clock_2_stopped;
  Globals::clock_2_stopped = true;

  // new section dealing with Snt & Snt-sh additions
  // BUT don't add trains if points or route flashing [conditions added for
  // Version 0.6 as a result of Najamuddin's error - 15/01/11] - wait until next
  // clock tick after stops flashing
  if (!Track::track->route_flash_flag && !Track::track->point_flash_flag) {
    for (unsigned int x{0}; x < train_data.size(); x++) {
      const Actions::TrainDataEntry &td_entry_{train_data.at(x)};

      const Actions::VectorEntry &av_entry_0_ = td_entry_.actions.at(0);
      const Actions::EventType evt_type_{Actions::EventType::NoEvent};

      if (av_entry_0_.command == "Snt") {
        // calc below only for Snt & Snt-sh entries rather than all entries to
        // save time
        const Actions::VectorEntry &av_entry_last_{
            td_entry_.actions.at(td_entry_.actions.size() - 1)};
        int incremental_mins_{0};
        int incremental_digits_{0};
        if (av_entry_last_.format_type == Timetable::FormatType::Repeat) {
          incremental_mins_ = av_entry_last_.rear_start_or_repeat_mins;
          incremental_digits_ = av_entry_last_.front_start_or_repeat_digits;
        }
        if ((av_entry_last_.format_type == Timetable::FormatType::Repeat) &&
            (td_entry_.n_trains < 2)) {
          throw std::runtime_error(
              "Repeat entry && less than two trains for Snt entry: " +
              td_entry_.head_code);
        }
        // see above note

        for (int y = 0; y < td_entry_.n_trains; y++) {
          Actions::TrainOperatingData ttod_{td_entry_.train_operating_data.at(y)};

          if (ttod_.running_entry != Actions::RunningEntry::NotStarted)
            continue;

          // Multiplayer: here check for a train entering at a coupling
          // {RearStartOrRepeatMins shows if it's a coupling or not), and can
          // only be a Snt entry if so and no arrival signalled yet bypass the
          // timetabled arrival if so and arrival signalled then start the new
          // service, using the repeat number and headcode for the entering
          // train if a repeat is skipped then should be ok if it arrives later
          // as its RunningEntry is still NotStarted

          boost::posix_time::time_duration av_0_evt_time_{
              av_entry_0_.event_time};

          if (getRepeatTime(2, av_0_evt_time_, y, incremental_mins_) >
              ttb_clock_time) {
            break; // all the rest will also be greater
          }

          const std::string train_head_code_ = getRepeatHeadCode(
              22, td_entry_.head_code, y, incremental_digits_);

          if (addTrain(2, av_entry_0_.rear_start_or_repeat_mins,
                       av_entry_0_.front_start_or_repeat_digits,
                       train_head_code_, td_entry_.start_speed, td_entry_.mass,
                       td_entry_.max_running_speed, td_entry_.max_brake_rate,
                       td_entry_.power_at_rail, "Timetable",
                       std::make_shared<Actions::TrainDataEntry>(td_entry_), y,
                       incremental_digits_, incremental_digits_,
                       td_entry_.signaller_speed,
                       av_entry_0_.under_signaller_control,
                       std::make_shared<Actions::EventType>(evt_type_))) {
            ttod_.train_id = trains.back().getID();
            ttod_.running_entry = Actions::RunningEntry::Running;
          } else if (evt_type_ == Actions::EventType::FailTrainEntry) {
            break; // if a train can't enter no point checking any more repeats
                   // as they won't be able to enter either
          }
        }
      }
      if (av_entry_0_.command == "Snt-sh")
      // just start this once, shuttle repeats take care of restarts
      {
        // calc below only for Snt & Snt-sh entries rather than all entries to
        // save time
        const Actions::VectorEntry &av_entry_last_ =
            td_entry_.actions.at(td_entry_.actions.size() - 1);
        int incremental_mins{0};
        int incremental_digits{0};
        if (av_entry_last_.format_type == Timetable::FormatType::Repeat) {
          incremental_mins = av_entry_last_.rear_start_or_repeat_mins;
          incremental_digits = av_entry_last_.front_start_or_repeat_digits;
        }
        if ((av_entry_last_.format_type == Timetable::FormatType::Repeat) &&
            (td_entry_.n_trains < 2)) {
          throw std::runtime_error("Repeat entry && less than two trains for "
                                   "Snt-sh entry: " +
                                   td_entry_.head_code);
        }
        // see above note
        Actions::TrainOperatingData ttod_{td_entry_.train_operating_data.at(0)};
        if (ttod_.running_entry == Actions::RunningEntry::NotStarted) {
          if (av_entry_0_.event_time <= ttb_clock_time) {
            if (addTrain(3, av_entry_0_.rear_start_or_repeat_mins,
                         av_entry_0_.front_start_or_repeat_digits,
                         td_entry_.head_code, td_entry_.start_speed,
                         td_entry_.mass, td_entry_.max_running_speed,
                         td_entry_.max_brake_rate, td_entry_.power_at_rail,
                         "Timetable",
                         std::make_shared<Actions::TrainDataEntry>(td_entry_),
                         0, incremental_mins, incremental_digits,
                         td_entry_.signaller_speed, false,
                         std::make_shared<Actions::EventType>(evt_type_)))
            // false for SignallerControl
            {
              ttod_.train_id = trains.back().getID();
              ttod_.running_entry = Actions::RunningEntry::Running;
            } else if (evt_type_ == Actions::EventType::FailTrainEntry) {
              break; // if a train can't enter no point checking any more
                     // repeats as they won't be able to enter either
            }
          }
        }
      }
    }
  }

  // deal with running trains but abort if any vectors added, would probably be
  // OK but don't risk a vector reallocation disrupting the iteration, next
  // cycle will catch up with any other pending updates
  if (!trains.empty()) {
    train_added = false;

    // elapsed time investigations

    // elapsed time  segment
    // double Start, End;
    // AnsiString ElapsedTimeReport = "";
    // end elasped time segment
    Track::all_routes->call_on_vector.clear(); // this will be rebuilt during the calls to
                                     // UpdateTrain elapsed time  segment
    // PerfLogForm->PerformanceLog(-1, "\n Train vector size: " +
    // AnsiString(TrainVector.size()) + '\n'); PerfLogForm->PerformanceLog(-1,
    // "Start time list"); end elapsed time  segment
    for (unsigned int x = 0; x < trains.size(); x++) {
      // elapsed time  segment
      // Start = double(GetTime()) * 86400; //secs
      // end elapsed time  segment
      trainVectorAt(33, x).updateTrain(0);
      // elapsed time  segment
      // End = double(GetTime()) * 86400;
      // ElapsedTimeReport = TrainVectorAt(-1,
      // x).TrainDataEntryPtr->ServiceReference + AnsiString(" ") +
      // AnsiString(int((End - Start) * 1000)); //msecs
      // PerfLogForm->PerformanceLog(-1, ElapsedTimeReport);
      // end elapsed time  segment

      // end elapsed time investigations

      /* added HasTrainGone() condition below in v0.4c to prevent 2 trains both
      having TrainGone set in UpdateTrain at the same time.  That caused the
      error Craig Weekes reported in November 2010 where 2 trains exited at the
      same time, and later the TrainVector iterates in reverse to erase the
      second train to have gone (when the first train to have gone comes before
      the second in TrainVector), but afterwards ReplotTrains iterates forwards
      and therefore replots the first train to have gone and therefore sets the
      TrainIDOnElement value to the exited train, with nothing to reset it.
      Hovering the mouse over that element with train information enabled causes
      an error because the track element thinks the train is still there,
      whereas it is missing from the TrainVector.  BUT subsequently (in v2.11.1)
      changed RePlotTrains so it doesn't plot trains with TrainGone set, but
      left this is as does no harm Had another error notified by Kevin Smith on
      02/01/22 where a train was manually removed in the same clock cycle as a
      train exited, and this caused the same error as above.  Did a lot of
      experimenting but eventually cured it with two changes, first as above in
      RePlotTrains, and also below adding a break; command after one
      TrainHasGone() dealt with.  There were introduced in v2.11.1 & seems ok
      now These changes should deal with any number of TrainGone flags set in
      the same clock cycle - from exiting, manual removal, or joins
      */
      if (train_added || trainVectorAt(35, x).hasTrainGone()) {
        break; // only one exited train will be dealt with at a time (see below)
               // so no point looking further
      }
    }
    // set warning flags
    crash_warning = false;
    derail_warning = false;
    spad_warning = false;
    call_on_warning = false;
    signal_stop_warning = false;
    buffer_attention_warning = false;
    train_failed_warning = false;
  
    for (int x{static_cast<int>(trains.size()) - 1}; x >= 0;
         x--) // reverse because of erase
    {
      Train &train_ = trainVectorAt(34, x);
      // if (Train.hasCrashed())
      // // can't use background colours for crashed & derailed because same colour
      // {
      //   crash_warning = true;
      // } else if (Train.hasDerailed())
      // // can't use background colours for crashed & derailed because same colour
      // {
      //   derail_warning = true;
      // } else if (Train.BackgroundColour == clSPADBackground)
      // // use colour as that changes as soon as passes signal
      // {
      //   SPADWarning = true;
      // } else if (Train.BackgroundColour == clTrainFailedBackground) {
      //   TrainFailedWarning = true;
      // } else if (Train.BackgroundColour == clCallOnBackground)
      // // use colour as also stopped at signal
      // {
      //   CallOnWarning = true;
      // } else if (Train.BackgroundColour == clSignalStopBackground)
      // // use colour to distinguish from call-on
      // {
      //   SignalStopWarning = true;
      // } else if (Train.BackgroundColour == clBufferAttentionNeeded)
      // // use colour to distinguish from ordinary buffer stop
      // {
      //   BufferAttentionWarning = true;
      // }
      if (train_.hasTrainGone()) {
        std::string loc_;
        bool element_found_{false};
        Track::TrackElement te_;
        if (train_.getLagElement() > -1) {
          te_ = Track::track->trackElementAt(531, train_.getLagElement());
          element_found_ = true;
        } else if (train_.getMidElement() > -1) {
          te_ = Track::track->trackElementAt(779, train_.getMidElement());
          element_found_ = true;
        } else if (train_.GetLeadElement() > -1) {
          te_ = Track::track->trackElementAt(780, train_.GetLeadElement());
          element_found_ = true;
        }
        if (element_found_) {
          if (te_.active_track_element_name != "") {
            loc_ = te_.active_track_element_name + ", track element " + te_.element_id;
          } else {
            loc_ = "track element " + te_.element_id;
          }
        }
        std::shared_ptr<Actions::VectorEntry> av_entry_ptr = train_.action_vector_entry;
        std::string other_headcode_;
        boost::posix_time::time_duration t_0_{0, 0, 0, 0};
        if ((train_.signaller_removed) || (train_.joined_other_train_flag))
        // need above first because may also have ActionVectorEntryPtr == "Fer"
        {
          train_.unplotTrain(9);
          // added at v1.3.0 to reset signals after train removed from an
          // autosigsroute
          if (signaller_train_removed_on_auto_sigs_route) {
            Track::all_routes->siganller_removed_train_auto_route.setRouteSignals(9);
            signaller_train_removed_on_auto_sigs_route = false;
          }
          // end of addition
          Track::all_routes->rebuild_railway_flag = true;
          // to force ClearandRebuildRailway at next clock tick if not in
          // zoom-out mode, to replot LCs correctly after a crash
        } else if (av_entry_ptr->command == "Fer") {
          bool correct_exit_{false};
          if (!av_entry_ptr->exit_list.empty()) {
            for (Containers::NumListIterator elit_{av_entry_ptr->exit_list.begin()};
                 elit_ != av_entry_ptr->exit_list.end(); elit_++) {
              if (*elit_ == train_.getLagElement()) {
                correct_exit_ = true;
              }
            }
          }
          if (correct_exit_) {
            train_.logAction(19, train_.getHeadCode(), other_headcode_, Actions::Type::Leave, loc_,
                            av_entry_ptr->event_time, av_entry_ptr->trigger_warning_panel_alert);
          } else {
            logActionError(38, train_.getHeadCode(), other_headcode_, Actions::EventType::FailIncorrectExit, loc_);
          }
        } else {
          if (!av_entry_ptr->under_signaller_control) {
            logActionError(26, train_.getHeadCode(), other_headcode_, Actions::EventType::FailUnexpectedExitRailway,
                           loc_);
            train_.sendMissedActionLogs(2, -2, av_entry_ptr);
            // -2 is marker for send messages for all remaining actions except
            // Fer if present
          } else {
            train_.logAction(31, train_.getHeadCode(), other_headcode_, Actions::Type::SignallerLeave, loc_,
                            t_0_, false); // false for Warning
          }
        }
        Utilities::cumulative_delayed_rand_mins_all_trains +=
            train_.getCumulativeDelayedRandMinsOneTrain(); // added at v2.13.0 for
                                                     // random delays
        train_.train_data_entry->train_operating_data.at(train_.getRepeatNumber())
            .running_entry = Actions::RunningEntry::Exited;
        train_.deleteTrain(1);
        trains.erase(trains.begin() + x);
        // replotTrains(1,
        //              Display); // to reset ElementIDs for remaining trains when
                               // have removed a train NB: won't plot any trains
                               // with TrainGone flag set (changed at v2.11.1)
        break; // added at v2.11.1 to ensure that only one train with TrainGone
               // set is dealt with in one clock cycle
      }
    }
  } else {
    // reset all flags in case last train removed with flag set
    crash_warning = false;
    derail_warning = false;
    spad_warning = false;
    call_on_warning = false;
    signal_stop_warning = false;
    buffer_attention_warning = false;
    train_failed_warning = false;
  }
  // update OpTimeToActMultimap
  if ((op_time_to_act_update_counter == 0) &&
      op_action_panel_visible) // new v2.2.0
  {
    rebuildOpTimeToActMultimap(0);
    // clears entries then adds values for running trains then for continuation
    // entries
    rebuildTimeToExitMultiMap(0);
    // added for multiplayer for running trains only
  }
  Utilities::clock_2_stopped = clock_state_;
  Logging::popCallLog(723);
}

bool TrainController::addTrain(int caller, int rear_position, int front_position, std::string head_code, int start_speed, int mass, double max_running_speed, double MaxBrakeRate,
                  double power_at_rail, std::string mode_str, std::shared_ptr<Actions::TrainDataEntry> train_data, int repear_number, int incremental_mins, int incremental_digits,
                  int signaller_speed, bool signaller_control, std::shared_ptr<Actions::EventType> event_type) {

    logEvent(std::to_string(caller) + ",AddTrain," + std::to_string(rear_position) + "," + std::to_string(front_position) + "," + head_code + "," + std::to_string(start_speed) +
             "," + std::to_string(mass) + "," + mode_str);

    Logging::call_log.push_back(Utilities::getTimeStamp() + "," + std::to_string(caller) + ",AddTrain," + std::to_string(rear_position) + "," + std::to_string(front_position) +
                                 "," + head_code + "," + std::to_string(start_speed) + "," + std::to_string(mass) + "," + mode_str); //at v2.11.1 dropped later headcode - was listed twice

    int rear_exit_pos_{-1};

    for(int x{0}; x < 4; x++)
    {
        if(Track->TrackElementAt(519, RearPosition).Conn[x] == FrontPosition)
        {
            RearExitPos = x;
        }
    }
    if(RearExitPos == -1)
    {
        throw Exception("Error, RearExit == -1 in AddTrain");
    }
    bool ReportFlag = true;

    // used to stop repeated messages from CheckStartAllowable when split failed
    if(TrainDataEntryPtr->TrainOperatingDataVector.at(RepeatNumber).EventReported != NoEvent)
    {
        ReportFlag = false;
    }
    if(!CheckStartAllowable(0, RearPosition, RearExitPos, HeadCode, ReportFlag, EventType))
    {
        // messages sent to performance log in CheckStartAllowable if ReportFlag true
        TrainDataEntryPtr->TrainOperatingDataVector.at(RepeatNumber).EventReported = EventType;
        Utilities->CallLogPop(938);
        return(false);
    }
    TrainDataEntryPtr->TrainOperatingDataVector.at(RepeatNumber).EventReported = NoEvent;
    TTrainMode TrainMode = NoMode;

    if(ModeStr == "Timetable")
    {
        TrainMode = Timetable;
    }
    // all else gives 'None', 'Signaller' set within program

    if(MaxRunningSpeed < 10)
    {
        MaxRunningSpeed = 10; // added at v0.6 to avoid low max speeds
    }
    if(SignallerSpeed < 10)
    {
        SignallerSpeed = 10; // added at v0.6 to avoid low max speeds
    }
    TTrain *NewTrain = new TTrain(0, RearPosition, RearExitPos, HeadCode, StartSpeed, Mass, MaxRunningSpeed, MaxBrakeRate, PowerAtRail, TrainMode,
                                  TrainDataEntryPtr, RepeatNumber, IncrementalMinutes, IncrementalDigits, SignallerSpeed);

    LogEvent("AddTrainSupplemental: Service Ref = " + TrainDataEntryPtr->ServiceReference + ", TrainID = " + AnsiString(NewTrain->TrainID)); //new at v2.11.1 so can relate headcode to ID

    NewTrain->ActionVectorEntryPtr = &(TrainDataEntryPtr->ActionVector.at(0));
    // initialise here rather than in TTrain constructor as create trains
    // with Null TrainDataEntryPtr when loading session trains
    if(SignallerControl)
    {
        NewTrain->TimetableFinished = true;
        NewTrain->SignallerStoppingFlag = false;
        NewTrain->TrainMode = Signaller;
        if(NewTrain->MaxRunningSpeed > NewTrain->SignallerMaxSpeed)
        {
            NewTrain->MaxRunningSpeed = NewTrain->SignallerMaxSpeed;
        }
        RailGraphics->ChangeForegroundColour(17, NewTrain->HeadCodePosition[0], NewTrain->FrontCodePtr, clFrontCodeSignaller, NewTrain->BackgroundColour);
    }
    // deal with starting conditions:-
    // unlocated Snt: just report entry & advance pointer
    // located Snt or Sfs: set station conditions as would if had reached stop point in Update(), & advance the ActionVectorEntryPtr
    // Sns doesn't need a new train
    if(NewTrain->ActionVectorEntryPtr->LocationName != "")
    // covers all above located starts
    // if location of Snt was a station (that is set as LocationName, i.e. not just any station) that isn't next departure station then
    // wouldn't have accepted the timetable
    {
        // first check if LeadElement (can't access LeadElement directly yet as not set, use FrontPosition instead) is buffers, note that
        // StoppedAtBuffers is set in UpdateTrain()
        if(Track->TrackElementAt(520, FrontPosition).TrackType == Buffers)
        // buffer end must be ahead of train or would have failed start position check
        {
            NewTrain->StoppedAtLocation = true;
            NewTrain->PlotStartPosition(0);
            NewTrain->PlotTrainWithNewBackgroundColour(13, clStationStopBackground, Display); // pale green
            NewTrain->LogAction(20, NewTrain->HeadCode, "", Create, NewTrain->ActionVectorEntryPtr->LocationName, NewTrain->ActionVectorEntryPtr->EventTime,
                                NewTrain->ActionVectorEntryPtr->Warning);
            if(!SignallerControl) // don't advance if SignalControlEntry
            {
                NewTrain->ActionVectorEntryPtr++;
                // should be a command, could be a location departure but if so can't depart so set 'Hold' anyway
            }
            NewTrain->LastActionTime = TTClockTime;
        }
        // else a through station stop
        else
        {
            NewTrain->StoppedAtLocation = true;
            NewTrain->PlotStartPosition(10);
            NewTrain->PlotTrainWithNewBackgroundColour(18, clStationStopBackground, Display); // pale green
            NewTrain->LogAction(21, NewTrain->HeadCode, "", Create, NewTrain->ActionVectorEntryPtr->LocationName, NewTrain->ActionVectorEntryPtr->EventTime,
                                NewTrain->ActionVectorEntryPtr->Warning);
            if(!SignallerControl) // don't advance if SignalControlEntry
            {
                NewTrain->ActionVectorEntryPtr++;
            }
            NewTrain->LastActionTime = TTClockTime;
        }
    }
    else // unlocated entry (i.e. not a stop entry, but could still be at a named location)
    {
        NewTrain->PlotStartPosition(11);
        TTrackElement TE = Track->TrackElementAt(530, NewTrain->RearStartElement);
        AnsiString Loc = "";
        if(TE.ActiveTrackElementName != "")
        {
            Loc = TE.ActiveTrackElementName + ", track element " + TE.ElementID;
        }
        else
        {
            Loc = "track element " + TE.ElementID;
        }
        if(TE.TrackType == Continuation)
        {
            NewTrain->LogAction(22, NewTrain->HeadCode, "", Enter, Loc, NewTrain->ActionVectorEntryPtr->EventTime, NewTrain->ActionVectorEntryPtr->Warning);
        }
        else
        {
            NewTrain->LogAction(23, NewTrain->HeadCode, "", Create, Loc, NewTrain->ActionVectorEntryPtr->EventTime, NewTrain->ActionVectorEntryPtr->Warning);
        }
        if(!SignallerControl) // don't advance if SignalControlEntry
        {
            NewTrain->ActionVectorEntryPtr++;
        }
        NewTrain->LastActionTime = TTClockTime;
        // no need to set LastActionTime for an unlocated entry
    }
    // cancel a wrong-direction route if either element of train starts on one
    if(NewTrain->LeadElement > -1)
    {
        NewTrain->CheckAndCancelRouteForWrongEndEntry(3, NewTrain->LeadElement, NewTrain->LeadEntryPos);
    }
    if(NewTrain->MidElement > -1)
    {
        NewTrain->CheckAndCancelRouteForWrongEndEntry(4, NewTrain->MidElement, NewTrain->MidEntryPos);
    }
    // set signals for a right-direction autosigs route for either element of train on one
    // erase elements back to start for a non-autosigs route & check if an autosigs route immediately behind it, and if so set its signals
    // note that all but autosigs routes become part of a single route, so there can only be an autosigs route behind the non-autosigs route
    int RouteNumber = -1;
    bool SignalsSet = false;

    if(NewTrain->LeadElement > -1)
    {
        if(AllRoutes->GetRouteTypeAndNumber(13, NewTrain->LeadElement, NewTrain->LeadEntryPos, RouteNumber) == TAllRoutes::AutoSigsRoute)
        {
            // below added in place of SetRouteSignals in v2.4.0 as don't want to set signals from start of route for a new train addition
            int RouteStartPosition;
            TAllRoutes::TRouteElementPair FirstPair, SecondPair;
            FirstPair = AllRoutes->GetRouteElementDataFromRoute2MultiMap(21, Track->TrackElementAt(955, FrontPosition).HLoc,
                                                                         Track->TrackElementAt(956, FrontPosition).VLoc, SecondPair);
            if(FirstPair.first == RouteNumber)
            {
                RouteStartPosition = FirstPair.second;
            }
            else if(SecondPair.first == RouteNumber)
            {
                RouteStartPosition = SecondPair.second;
            }
            else
            {
                throw Exception("Error, RouteNumber not found in Route2MultiMap in 1st of 2 calls to SetAllRearwardsSignals in AddTrain");
            }
            AllRoutes->SetAllRearwardsSignals(10, 0, RouteNumber, RouteStartPosition);
            SignalsSet = true;
            // AllRoutes->GetFixedRouteAt(, RouteNumber).SetRouteSignals();  above substituted in v2.4.0
        }
        else if(RouteNumber > -1) // non-autosigsroute
        {
            TPrefDirElement TempPDE = AllRoutes->GetFixedRouteAt(181, RouteNumber).GetFixedPrefDirElementAt(194, 0);
            int FirstTVPos = TempPDE.GetTrackVectorPosition();
            int FirstELinkPos = TempPDE.GetELinkPos();
            while(TempPDE.GetTrackVectorPosition() != (unsigned int)(NewTrain->LeadElement))
            {
                AllRoutes->RemoveRouteElement(16, TempPDE.HLoc, TempPDE.VLoc, TempPDE.GetELink());
                TempPDE = AllRoutes->GetFixedRouteAt(182, RouteNumber).GetFixedPrefDirElementAt(195, 0);
            }
            if(TempPDE.GetTrackVectorPosition() == (unsigned int)(NewTrain->LeadElement))
            {
                AllRoutes->RemoveRouteElement(17, TempPDE.HLoc, TempPDE.VLoc, TempPDE.GetELink());
                // remove the last element under LeadElement
            }
            AllRoutes->RebuildRailwayFlag = true;
            // to force ClearandRebuildRailway at next clock tick if not in zoom-out mode
            // now deal with a rear linked autosigs route
            if(Track->TrackElementAt(820, FirstTVPos).Conn[FirstELinkPos] > -1)
            {
                int LinkedRouteNumber = -1;
                if(AllRoutes->GetRouteTypeAndNumber(17, Track->TrackElementAt(821, FirstTVPos).Conn[FirstELinkPos],
                                                    Track->TrackElementAt(822, FirstTVPos).ConnLinkPos[FirstELinkPos], LinkedRouteNumber) == TAllRoutes::AutoSigsRoute)
                {
                    AllRoutes->GetFixedRouteAt(169, LinkedRouteNumber).SetRouteSignals(0);
                    // this is ok as here we are setting signals from the start of the route
                }
            }
            SignalsSet = true;
        }
    }
    if(NewTrain->MidElement > -1)
    // if entering at a continuation MidElement == -1
    {
        // this is included in case a train starts with LeadElement on no route and MidElement on a route
        if(!SignalsSet)
        {
            RouteNumber = -1;
            if(AllRoutes->GetRouteTypeAndNumber(14, NewTrain->MidElement, NewTrain->MidEntryPos, RouteNumber) == TAllRoutes::AutoSigsRoute)
            {
                // below added in place of SetRouteSignals in v2.4.0 as don't want to set signals from start of route for a new train addition
                int RouteStartPosition;
                TAllRoutes::TRouteElementPair FirstPair, SecondPair;
                FirstPair = AllRoutes->GetRouteElementDataFromRoute2MultiMap(22, Track->TrackElementAt(957, RearPosition).HLoc,
                                                                             Track->TrackElementAt(958, RearPosition).VLoc, SecondPair);
                if(FirstPair.first == RouteNumber)
                {
                    RouteStartPosition = FirstPair.second;
                }
                else if(SecondPair.first == RouteNumber)
                {
                    RouteStartPosition = SecondPair.second;
                }
                else
                {
                    throw Exception("Error, RouteNumber not found in Route2MultiMap in 2nd of 2 calls to SetAllRearwardsSignals in AddTrain");
                }
                AllRoutes->SetAllRearwardsSignals(11, 0, RouteNumber, RouteStartPosition);
                SignalsSet = true;
                // AllRoutes->GetFixedRouteAt(, RouteNumber).SetRouteSignals();  above substituted in v2.4.0
            }
            else if(RouteNumber > -1) // non-autosigsroute
            {
                TPrefDirElement TempPDE = AllRoutes->GetFixedRouteAt(184, RouteNumber).GetFixedPrefDirElementAt(196, 0);
                int FirstTVPos = TempPDE.GetTrackVectorPosition();
                int FirstELinkPos = TempPDE.GetELinkPos();
                while(TempPDE.GetTrackVectorPosition() != (unsigned int)(NewTrain->MidElement))
                {
                    AllRoutes->RemoveRouteElement(18, TempPDE.HLoc, TempPDE.VLoc, TempPDE.GetELink());
                    TempPDE = AllRoutes->GetFixedRouteAt(185, RouteNumber).GetFixedPrefDirElementAt(197, 0);
                }
                if(TempPDE.GetTrackVectorPosition() == (unsigned int)(NewTrain->MidElement))
                {
                    AllRoutes->RemoveRouteElement(19, TempPDE.HLoc, TempPDE.VLoc, TempPDE.GetELink());
                    // remove the last element under LeadElement
                }
                AllRoutes->RebuildRailwayFlag = true;
                // to force ClearandRebuildRailway at next clock tick if not in zoom-out mode
                // now deal with a rear linked autosigs route
                if(Track->TrackElementAt(823, FirstTVPos).Conn[FirstELinkPos] > -1)
                {
                    int LinkedRouteNumber = -1;
                    if(AllRoutes->GetRouteTypeAndNumber(19, Track->TrackElementAt(824, FirstTVPos).Conn[FirstELinkPos],
                                                        Track->TrackElementAt(825, FirstTVPos).ConnLinkPos[FirstELinkPos], LinkedRouteNumber) == TAllRoutes::AutoSigsRoute)
                    {
                        AllRoutes->GetFixedRouteAt(170, LinkedRouteNumber).SetRouteSignals(1);
                        // this is ok as now we are setting signals from the start of the route
                    }
                }
            }
        }
    }
    TrainVector.push_back(*NewTrain);
    Utilities->CallLogPop(731);
    return(true);
}

}; // namespace RailOS::Train