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
      const std::shared_ptr<Actions::TrainDataEntry> td_entry_{&train_data.at(x)};

      const Actions::VectorEntry &av_entry_0_ = td_entry_->actions.at(0);
      const Actions::EventType evt_type_{Actions::EventType::NoEvent};

      if (av_entry_0_.command == "Snt") {
        // calc below only for Snt & Snt-sh entries rather than all entries to
        // save time
        const Actions::VectorEntry &av_entry_last_{
            td_entry_->actions.at(td_entry_->actions.size() - 1)};
        int incremental_mins_{0};
        int incremental_digits_{0};
        if (av_entry_last_.format_type == Timetable::FormatType::Repeat) {
          incremental_mins_ = av_entry_last_.rear_start_or_repeat_mins;
          incremental_digits_ = av_entry_last_.front_start_or_repeat_digits;
        }
        if ((av_entry_last_.format_type == Timetable::FormatType::Repeat) &&
            (td_entry_->n_trains < 2)) {
          throw std::runtime_error(
              "Repeat entry && less than two trains for Snt entry: " +
              td_entry_->head_code);
        }
        // see above note

        for (int y = 0; y < td_entry_->n_trains; y++) {
          Actions::TrainOperatingData ttod_{
              td_entry_->train_operating_data.at(y)};

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
              22, td_entry_->head_code, y, incremental_digits_);

          if (addTrain(2, av_entry_0_.rear_start_or_repeat_mins,
                       av_entry_0_.front_start_or_repeat_digits,
                       train_head_code_, td_entry_->start_speed, td_entry_->mass,
                       td_entry_->max_running_speed, td_entry_->max_brake_rate,
                       td_entry_->power_at_rail, "Timetable",
                       td_entry_, y,
                       incremental_digits_, incremental_digits_,
                       td_entry_->signaller_speed,
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
            td_entry_->actions.at(td_entry_->actions.size() - 1);
        int incremental_mins{0};
        int incremental_digits{0};
        if (av_entry_last_.format_type == Timetable::FormatType::Repeat) {
          incremental_mins = av_entry_last_.rear_start_or_repeat_mins;
          incremental_digits = av_entry_last_.front_start_or_repeat_digits;
        }
        if ((av_entry_last_.format_type == Timetable::FormatType::Repeat) &&
            (td_entry_->n_trains < 2)) {
          throw std::runtime_error("Repeat entry && less than two trains for "
                                   "Snt-sh entry: " +
                                   td_entry_->head_code);
        }
        // see above note
        Actions::TrainOperatingData ttod_{td_entry_->train_operating_data.at(0)};
        if (ttod_.running_entry == Actions::RunningEntry::NotStarted) {
          if (av_entry_0_.event_time <= ttb_clock_time) {
            if (addTrain(3, av_entry_0_.rear_start_or_repeat_mins,
                         av_entry_0_.front_start_or_repeat_digits,
                         td_entry_->head_code, td_entry_->start_speed,
                         td_entry_->mass, td_entry_->max_running_speed,
                         td_entry_->max_brake_rate, td_entry_->power_at_rail,
                         "Timetable",
                         td_entry_,
                         0, incremental_mins, incremental_digits,
                         td_entry_->signaller_speed, false,
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
    Track::all_routes->call_on_vector
        .clear(); // this will be rebuilt during the calls to
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
      // // can't use background colours for crashed & derailed because same
      // colour
      // {
      //   crash_warning = true;
      // } else if (Train.hasDerailed())
      // // can't use background colours for crashed & derailed because same
      // colour
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
        } else if (train_.getLeadElement() > -1) {
          te_ = Track::track->trackElementAt(780, train_.getLeadElement());
          element_found_ = true;
        }
        if (element_found_) {
          if (te_.active_track_element_name != "") {
            loc_ = te_.active_track_element_name + ", track element " +
                   te_.element_id;
          } else {
            loc_ = "track element " + te_.element_id;
          }
        }
        std::shared_ptr<Actions::VectorEntry> av_entry_ptr =
            train_.action_vector_entry;
        std::string other_headcode_;
        boost::posix_time::time_duration t_0_{0, 0, 0, 0};
        if ((train_.signaller_removed) || (train_.joined_other_train_flag))
        // need above first because may also have ActionVectorEntryPtr == "Fer"
        {
          train_.unplotTrain(9);
          // added at v1.3.0 to reset signals after train removed from an
          // autosigsroute
          if (signaller_train_removed_on_auto_sigs_route) {
            Track::all_routes->siganller_removed_train_auto_route
                .setRouteSignals(9);
            signaller_train_removed_on_auto_sigs_route = false;
          }
          // end of addition
          Track::all_routes->rebuild_railway_flag = true;
          // to force ClearandRebuildRailway at next clock tick if not in
          // zoom-out mode, to replot LCs correctly after a crash
        } else if (av_entry_ptr->command == "Fer") {
          bool correct_exit_{false};
          if (!av_entry_ptr->exit_list.empty()) {
            for (Containers::NumListIterator elit_{
                     av_entry_ptr->exit_list.begin()};
                 elit_ != av_entry_ptr->exit_list.end(); elit_++) {
              if (*elit_ == train_.getLagElement()) {
                correct_exit_ = true;
              }
            }
          }
          if (correct_exit_) {
            train_.logAction(19, train_.getHeadCode(), other_headcode_,
                             Actions::Type::Leave, loc_,
                             av_entry_ptr->event_time,
                             av_entry_ptr->trigger_warning_panel_alert);
          } else {
            logActionError(38, train_.getHeadCode(), other_headcode_,
                           Actions::EventType::FailIncorrectExit, loc_);
          }
        } else {
          if (!av_entry_ptr->under_signaller_control) {
            logActionError(26, train_.getHeadCode(), other_headcode_,
                           Actions::EventType::FailUnexpectedExitRailway, loc_);
            train_.sendMissedActionLogs(2, -2, av_entry_ptr);
            // -2 is marker for send messages for all remaining actions except
            // Fer if present
          } else {
            train_.logAction(31, train_.getHeadCode(), other_headcode_,
                             Actions::Type::SignallerLeave, loc_, t_0_,
                             false); // false for Warning
          }
        }
        Utilities::cumulative_delayed_rand_mins_all_trains +=
            train_.getCumulativeDelayedRandMinsOneTrain(); // added at v2.13.0
                                                           // for random delays
        train_.train_data_entry->train_operating_data
            .at(train_.getRepeatNumber())
            .running_entry = Actions::RunningEntry::Exited;
        train_.deleteTrain(1);
        trains.erase(trains.begin() + x);
        // replotTrains(1,
        //              Display); // to reset ElementIDs for remaining trains
        //              when
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

bool TrainController::addTrain(
    int caller, int rear_position, int front_position, std::string head_code,
    int start_speed, int mass, double max_running_speed, double max_brake_rate,
    double power_at_rail, std::string mode_str,
    std::shared_ptr<Actions::TrainDataEntry> train_data, int repeat_number,
    int incremental_mins, int incremental_digits, int signaller_speed,
    bool signaller_control, std::shared_ptr<Actions::EventType> event_type) {

  logEvent(std::to_string(caller) + ",AddTrain," +
           std::to_string(rear_position) + "," +
           std::to_string(front_position) + "," + head_code + "," +
           std::to_string(start_speed) + "," + std::to_string(mass) + "," +
           mode_str);

  Logging::call_log.push_back(
      Utilities::getTimeStamp() + "," + std::to_string(caller) + ",AddTrain," +
      std::to_string(rear_position) + "," + std::to_string(front_position) +
      "," + head_code + "," + std::to_string(start_speed) + "," +
      std::to_string(mass) + "," +
      mode_str); // at v2.11.1 dropped later headcode - was listed twice

  int rear_exit_pos_{-1};

  for (int x{0}; x < 4; x++) {
    if (Track::track->trackElementAt(519, rear_position).conn[x] ==
        front_position) {
      rear_exit_pos_ = x;
    }
  }
  if (rear_exit_pos_ == -1) {
    throw std::runtime_error("Error, RearExit == -1 in AddTrain");
  }
  bool report_flag_ = true;

  // used to stop repeated messages from CheckStartAllowable when split failed
  if (train_data->train_operating_data.at(repeat_number).event_reported !=
      Actions::EventType::NoEvent) {
    report_flag_ = false;
  }
  if (!checkStartAllowable(0, rear_position, rear_exit_pos_, head_code,
                           report_flag_, event_type)) {
    // messages sent to performance log in CheckStartAllowable if ReportFlag
    // true
    train_data->train_operating_data.at(repeat_number).event_reported =
        *event_type;
    Logging::popCallLog(938);
    return false;
  }
  train_data->train_operating_data.at(repeat_number).event_reported =
      Actions::EventType::NoEvent;
  Mode train_mode_ = Mode::NoMode;

  if (mode_str == "Timetable") {
    train_mode_ = Mode::Timetable;
  }
  // all else gives 'None', 'Signaller' set within program

  if (max_running_speed < 10) {
    max_running_speed = 10; // added at v0.6 to avoid low max speeds
  }
  if (signaller_speed < 10) {
    signaller_speed = 10; // added at v0.6 to avoid low max speeds
  }
  std::shared_ptr<Train> new_train_ = std::shared_ptr<Train>(new Train(
      0, rear_position, rear_exit_pos_, head_code, start_speed, mass,
      max_running_speed, max_brake_rate, power_at_rail, train_mode_, train_data,
      repeat_number, incremental_mins, incremental_digits, signaller_speed));

  logEvent(
      "AddTrainSupplemental: Service Ref = " + train_data->service_reference +
      ", TrainID = " +
      std::to_string(
          new_train_->getID())); // new at v2.11.1 so can relate headcode to ID

  int av_loc_{0};
  std::string other_headcode_("");

  new_train_->action_vector_entry =
      std::shared_ptr<Actions::VectorEntry>(&train_data->actions.at(av_loc_));

  // initialise here rather than in TTrain constructor as create trains
  // with Null TrainDataEntryPtr when loading session trains
  if (signaller_control) {
    new_train_->timetable_finished = true;
    new_train_->signaller_stopping_flag = false;
    new_train_->train_mode = Mode::Signaller;
    if (new_train_->max_running_speed > new_train_->getSignallerMaxSpeed()) {
      new_train_->max_running_speed = new_train_->getSignallerMaxSpeed();
    }
    // RailGraphics->ChangeForegroundColour(
    //     17, NewTrain->HeadCodePosition[0], NewTrain->FrontCodePtr,
    //     clFrontCodeSignaller, NewTrain->BackgroundColour);
  }
  // deal with starting conditions:-
  // unlocated Snt: just report entry & advance pointer
  // located Snt or Sfs: set station conditions as would if had reached stop
  // point in Update(), & advance the ActionVectorEntryPtr Sns doesn't need a
  // new train
  if (new_train_->action_vector_entry->location_name != "")
  // covers all above located starts
  // if location of Snt was a station (that is set as LocationName, i.e. not
  // just any station) that isn't next departure station then wouldn't have
  // accepted the timetable
  {
    // first check if LeadElement (can't access LeadElement directly yet as not
    // set, use FrontPosition instead) is buffers, note that StoppedAtBuffers is
    // set in UpdateTrain()
    if (Track::track->trackElementAt(520, front_position).track_type ==
        Track::Type::Buffers)
    // buffer end must be ahead of train or would have failed start position
    // check
    {
      new_train_->stopped_at_location = true;

      new_train_->plotStartPosition(0);

      // new_train_->plotTrainWithNewBackgroundColour(13,
      // clStationStopBackground,
      //                                            Display); // pale green
      new_train_->logAction(
          20, new_train_->getHeadCode(), other_headcode_, Actions::Type::Create,
          new_train_->action_vector_entry->location_name,
          new_train_->action_vector_entry->event_time,
          new_train_->action_vector_entry->trigger_warning_panel_alert);
      if (!signaller_control) // don't advance if SignalControlEntry
      {
        av_loc_++;
        new_train_->action_vector_entry = std::shared_ptr<Actions::VectorEntry>(
            &train_data->actions.at(av_loc_));
        // should be a command, could be a location departure but if so can't
        // depart so set 'Hold' anyway
      }
      new_train_->last_action_time = ttb_clock_time;
    }
    // else a through station stop
    else {
      new_train_->stopped_at_location = true;
      new_train_->plotStartPosition(10);
      // new_train_->plotTrainWithNewBackgroundColour(18,
      // clStationStopBackground,
      //                                            Display); // pale green
      new_train_->logAction(
          21, new_train_->getHeadCode(), other_headcode_, Actions::Type::Create,
          new_train_->action_vector_entry->location_name,
          new_train_->action_vector_entry->event_time,
          new_train_->action_vector_entry->trigger_warning_panel_alert);
      if (!signaller_control) // don't advance if SignalControlEntry
      {
        av_loc_++;
        new_train_->action_vector_entry = std::shared_ptr<Actions::VectorEntry>(
            &train_data->actions.at(av_loc_));
      }
      new_train_->last_action_time = ttb_clock_time;
    }
  } else // unlocated entry (i.e. not a stop entry, but could still be at a
         // named location)
  {
    new_train_->plotStartPosition(11);
    Track::TrackElement te_ =
        Track::track->trackElementAt(530, new_train_->getRearStartElement());
    std::string location_("");

    if (te_.active_track_element_name != "") {
      location_ =
          te_.active_track_element_name + ", track element " + te_.element_id;
    } else {
      location_ = "track element " + te_.element_id;
    }
    if (te_.track_type == Track::Type::Continuation) {
      new_train_->logAction(
          22, new_train_->getHeadCode(), other_headcode_, Actions::Type::Create,
          new_train_->action_vector_entry->location_name,
          new_train_->action_vector_entry->event_time,
          new_train_->action_vector_entry->trigger_warning_panel_alert);
    } else {
      new_train_->logAction(
          23, new_train_->getHeadCode(), other_headcode_, Actions::Type::Create,
          location_, new_train_->action_vector_entry->event_time,
          new_train_->action_vector_entry->trigger_warning_panel_alert);
    }
    if (!signaller_control) // don't advance if SignalControlEntry
    {
      av_loc_++;
      new_train_->action_vector_entry = std::shared_ptr<Actions::VectorEntry>(
          &train_data->actions.at(av_loc_));
    }
    new_train_->last_action_time = ttb_clock_time;
    // no need to set LastActionTime for an unlocated entry
  }
  // cancel a wrong-direction route if either element of train starts on one
  if (new_train_->getLeadElement() > -1) {
    new_train_->checkAndCancelRouteForWrongEndEntry(
        3, new_train_->getLeadElement(), new_train_->getLeadEntryPosition());
  }
  if (new_train_->getMidElement() > -1) {
    new_train_->checkAndCancelRouteForWrongEndEntry(
        4, new_train_->getMidElement(), new_train_->getMidEntryPosition());
  }
  // set signals for a right-direction autosigs route for either element of
  // train on one erase elements back to start for a non-autosigs route & check
  // if an autosigs route immediately behind it, and if so set its signals note
  // that all but autosigs routes become part of a single route, so there can
  // only be an autosigs route behind the non-autosigs route
  int route_number_{-1};
  bool signals_set_{false};

  if (new_train_->getLeadElement() > -1) {
    if (Track::all_routes->getRouteTypeAndNumber(
            13, new_train_->getLeadElement(),
            new_train_->getLeadEntryPosition(),
            route_number_) == Track::AllRoutes::RouteType::AutoSigsRoute) {
      // below added in place of SetRouteSignals in v2.4.0 as don't want to set
      // signals from start of route for a new train addition
      int route_start_position_{-1};

      Track::AllRoutes::RouteElementPair first_pair_, second_pair_;

      first_pair_ = Track::all_routes->getRouteElementDataFromRoute2MultiMap(
          21, Track::track->trackElementAt(955, front_position).h_loc,
          Track::track->trackElementAt(956, front_position).v_loc,
          second_pair_);
      if (first_pair_.first == route_number_) {
        route_start_position_ = first_pair_.second;
      } else if (second_pair_.first == route_number_) {
        route_start_position_ = second_pair_.second;
      } else {
        throw std::runtime_error(
            "Error, RouteNumber not found in Route2MultiMap in 1st "
            "of 2 calls to SetAllRearwardsSignals in AddTrain");
      }
      Track::all_routes->setAllRearwardsSignals(10, 0, route_number_,
                                                route_start_position_);
      signals_set_ = true;
      // AllRoutes->GetFixedRouteAt(, RouteNumber).SetRouteSignals();  above
      // substituted in v2.4.0
    } else if (route_number_ > -1) // non-autosigsroute
    {
      Track::PrefDirElement temp_pde_{
          Track::all_routes->getFixedRouteAt(181, route_number_)
              .getFixedPrefDirElementAt(194, 0)};

      int first_tv_pos_{static_cast<int>(temp_pde_.getTrackVectorPosition())};
      int first_elink_pos_{temp_pde_.getELinkPos()};

      while (temp_pde_.getTrackVectorPosition() !=
             static_cast<unsigned int>(new_train_->getLeadElement())) {
        Track::all_routes->removeRouteElement(
            16, temp_pde_.h_loc, temp_pde_.v_loc, temp_pde_.getELink());
        temp_pde_ = Track::all_routes->getFixedRouteAt(182, route_number_)
                        .getFixedPrefDirElementAt(195, 0);
      }
      if (temp_pde_.getTrackVectorPosition() ==
          static_cast<unsigned int>(new_train_->getLeadElement())) {
        Track::all_routes->removeRouteElement(
            17, temp_pde_.h_loc, temp_pde_.v_loc, temp_pde_.getELink());
        // remove the last element under LeadElement
      }
      Track::all_routes->rebuild_railway_flag = true;
      // to force ClearandRebuildRailway at next clock tick if not in zoom-out
      // mode now deal with a rear linked autosigs route
      if (Track::track->trackElementAt(820, first_tv_pos_)
              .conn[first_elink_pos_] > -1) {
        int linked_route_number_{-1};
        if (Track::all_routes->getRouteTypeAndNumber(
                17,
                Track::track->trackElementAt(821, first_tv_pos_)
                    .conn[first_elink_pos_],
                Track::track->trackElementAt(822, first_tv_pos_)
                    .con_link_pos[first_elink_pos_],
                linked_route_number_) ==
            Track::AllRoutes::RouteType::AutoSigsRoute) {
          Track::all_routes->getFixedRouteAt(169, linked_route_number_)
              .setRouteSignals(0);
          // this is ok as here we are setting signals from the start of the
          // route
        }
      }
      signals_set_ = true;
    }
  }
  if (new_train_->getMidElement() > -1)
  // if entering at a continuation MidElement == -1
  {
    // this is included in case a train starts with LeadElement on no route and
    // MidElement on a route
    if (!signals_set_) {
      route_number_ = -1;
      if (Track::all_routes->getRouteTypeAndNumber(
              14, new_train_->getMidElement(),
              new_train_->getMidEntryPosition(),
              route_number_) == Track::AllRoutes::RouteType::AutoSigsRoute) {
        // below added in place of SetRouteSignals in v2.4.0 as don't want to
        // set signals from start of route for a new train addition
        int route_start_position_{-1};
        Track::AllRoutes::RouteElementPair first_pair_, second_pair_;

        first_pair_ = Track::all_routes->getRouteElementDataFromRoute2MultiMap(
            22, Track::track->trackElementAt(957, rear_position).h_loc,
            Track::track->trackElementAt(958, rear_position).v_loc,
            second_pair_);
        if (first_pair_.first == route_number_) {
          route_start_position_ = first_pair_.second;
        } else if (second_pair_.first == route_number_) {
          route_start_position_ = second_pair_.second;
        } else {
          throw std::runtime_error(
              "Error, RouteNumber not found in Route2MultiMap in 2nd of 2 "
              "calls to SetAllRearwardsSignals in AddTrain");
        }
        Track::all_routes->setAllRearwardsSignals(11, 0, route_number_,
                                                  route_start_position_);
        signals_set_ = true;
        // AllRoutes->GetFixedRouteAt(, RouteNumber).SetRouteSignals();  above
        // substituted in v2.4.0
      } else if (route_number_ > -1) // non-autosigsroute
      {
        Track::PrefDirElement temp_pde_{
            Track::all_routes->getFixedRouteAt(184, route_number_)
                .getFixedPrefDirElementAt(196, 0)};

        int first_tv_pos_{static_cast<int>(temp_pde_.getTrackVectorPosition())};
        int first_elink_pos_{temp_pde_.getELinkPos()};

        while (temp_pde_.getTrackVectorPosition() !=
               static_cast<unsigned int>(new_train_->getMidElement())) {
          Track::all_routes->removeRouteElement(
              18, temp_pde_.h_loc, temp_pde_.v_loc, temp_pde_.getELink());
          temp_pde_ = Track::all_routes->getFixedRouteAt(185, route_number_)
                          .getFixedPrefDirElementAt(197, 0);
        }
        if (temp_pde_.getTrackVectorPosition() ==
            static_cast<unsigned int>(new_train_->getMidElement())) {
          Track::all_routes->removeRouteElement(
              19, temp_pde_.h_loc, temp_pde_.v_loc, temp_pde_.getELink());
          // remove the last element under LeadElement
        }
        Track::all_routes->rebuild_railway_flag = true;
        // to force ClearandRebuildRailway at next clock tick if not in zoom-out
        // mode now deal with a rear linked autosigs route
        if (Track::track->trackElementAt(823, first_tv_pos_)
                .conn[first_elink_pos_] > -1) {
          int linked_route_number_{-1};
          if (Track::all_routes->getRouteTypeAndNumber(
                  19,
                  Track::track->trackElementAt(824, first_tv_pos_)
                      .conn[first_elink_pos_],
                  Track::track->trackElementAt(825, first_tv_pos_)
                      .con_link_pos[first_elink_pos_],
                  linked_route_number_) ==
              Track::AllRoutes::RouteType::AutoSigsRoute) {
            Track::all_routes->getFixedRouteAt(170, linked_route_number_)
                .setRouteSignals(1);
            // this is ok as now we are setting signals from the start of the
            // route
          }
        }
      }
    }
  }
  trains.push_back(*new_train_);
  Logging::popCallLog(731);
  return true;
}

}; // namespace RailOS::Train