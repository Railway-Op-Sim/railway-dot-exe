#pragma once

#include <vector>
#include <map>

#include "boost/date_time/posix_time/posix_time_config.hpp"

#include "railos/actions.hxx"
#include "railos/containers.hxx"
#include "railos/train.hxx"
#include "railos/utilities.hxx"
#include "railos/logging.hxx"
#include "railos/timetable.hxx"
#include "railos/globals.hxx"
#include "railos/track.hxx"

namespace RailOS::Train {
/// Handles all train and timetable activities, only one object created
class TrainController
{
public:

    boost::posix_time::time_duration base_time{0, 0, 0, 0};
///< CurrentDateTime (i.e. real time) when operation restarts after a pause
    boost::posix_time::time_duration ttb_clock_time{0, 0, 0, 0};
///< the time indicated by the timetable clock
    boost::posix_time::time_duration ttb_start_time{0, 0, 0, 0};
///< the start time of the current timetable
    boost::posix_time::time_duration restart_time{0, 0, 0, 0};
///< TTClockTime when operation pauses ( = timetable start time prior to operation) TTClockTime is calculated as follows:- TTClockTime = CurrentDateTime + RestartTime - BaseTime;
    boost::posix_time::time_duration session_save_ttb_clock_time{0, 0, 0, 0}; //added at v2.5.0
///< TTClockTime when last session saved - to prevent display of warning message on exit session if < 5 minutes ago

/// Turns signals back to green in stages after a train has exited an autosig route at a continuation
    class ContinuationAutoSigEntry
    {
    public:
        double first_delay{0}, second_delay{0}, third_delay{0};
///< Delays in seconds before consecutive signal changes - these correspond to the times taken for trains to pass subsequent signals outside the boundaries of the railway.  After the third delay the signal nearest to the continuation that was red when the train passed it has changed to green
        int access_number{0};
///< the number of times the signal changing function has been accessed - starts at 0 and increments after each change
        int route_number{0};
///< the AllRoutesVector position of the route
        boost::posix_time::time_duration exit_time;
///< the timetable clock time at which the train exits from the continuation
    };

    typedef std::vector<ContinuationAutoSigEntry>ContinuationAutoSigVector;
///< vector class for TContinuationAutoSigEntry objects
    typedef ContinuationAutoSigVector::iterator ContinuationAutoSigVectorIterator;

/// Class that stores data for trains expected at continuation entries (kept in a multimap - see below), used to display information in the floating window when mouse hovers over a continuation
    class ContinuationTrainExpectationEntry
    {
    public:
        std::string description;
///< service description
        std::string head_code;
///< service headcode
        int repeat_number{0};
///< service RepeatNumber
        int incremental_mins{0};
///< Repeat separation in minutes
        int incremental_digits{0};
///< Repeat headcode separation
        int vector_pos{0};
///< TrackVectorPosition for the continuation element
        std::shared_ptr<Actions::TrainDataEntry> train_data_entry;
///< points to the service entry in the timetable's TrainDataVector
    };

    typedef std::multimap<boost::posix_time::time_duration, ContinuationTrainExpectationEntry> ContinuationTrainExpectationMultiMap;
///< Multimap class for TContinuationTrainExpectationEntry objects, where the access key is the expectation time
    typedef ContinuationTrainExpectationMultiMap::iterator ContinuationTrainExpectationMultiMapIterator;
///< iterator for the multimap
    typedef std::pair<boost::posix_time::time_duration, ContinuationTrainExpectationEntry>ContinuationTrainExpectationMultiMapPair;
///< a single multimap entry
    typedef std::vector<Train>TrainVector;
///< vector containing all trains that currently exist in the railway

/// Class used for timetable conflict file compilation
    struct LocServiceTimes
    {
        std::string location;
        std::string service_and_repeat_num;
        std::string at_loc_time;
        std::string arr_time;
        std::string dep_time;
        std::string frh_marker;
    };

    typedef std::vector<LocServiceTimes> LocServiceTimesVector;

/// Used in determining train directions in timetable conflict analysis
    typedef std::list<std::string> ServiceCallingLocsList;
/// Map of all ServiceCallingLocsLists with key as service reference.  A map is ok as all service refs are unique when it is built
    typedef std::map<std::string, ServiceCallingLocsList> AllServiceCallingLocsMap;
    AllServiceCallingLocsMap all_services_calling_locs_map;
// these added v2.2.0, for OpTimeToActPanel
    typedef std::pair<std::string, int>HCandTrainPosParam;
/// Headcode + TrainID for running trains & -TrackVectorPosition - 1 for continuation.
/** entries neg values for continuation track vector & -1 as could be 0 so all negative.  Was simply the trackvector (TV) position, but
since OA panel only rebuilt every 2 secs when mouseup on panel the train could be well past the TV position stored, especially at
16x operating speed*/
    typedef std::multimap<float, HCandTrainPosParam>OpTimeToActMultiMap;
    typedef std::pair<float, HCandTrainPosParam>OpTimeToActMultiMapEntry;
    typedef OpTimeToActMultiMap::iterator OpTimeToActMultiMapIterator;
    typedef std::vector<int>TContinuationEntryVecPosVector;
///<ensures only one train displayed for a given continuation

    std::string last_ttb_time;
///< Stores the last time used in the timetable as an AnsiString - used for timetable analysis
    std::string service_reference;
///< String used to display the offending service in timetable error messages
    bool crash_warning{false}, derail_warning{false}, spad_warning{false}, call_on_warning{false}, signal_stop_warning{false}, buffer_attention_warning{false}, train_failed_warning{false};
///< Flags to enable the relevant warning graphics to flash at the left hand side of the screen
    bool stop_ttb_clock{false};
///< when true the timetable clock is stopped, used for messages display and train popup menu display etc
    bool train_added{false};
///< true when a train has been added by a split (occurs outside the normal train introduction process)
    bool signaller_train_removed_on_auto_sigs_route{false};
///< true if train was on an AutoSigsRoute when removed by the signaller
    bool op_action_panel_visible{false};
///<new v2.2.0 flag to prevent time to act functions when not visible
    bool two_or_more_locations_warning_given{false};
///<new at v2.6.0 to allow loops
    bool ttb_edit_panel_visible{false};
///<new at v2.6.0 so potential error message only shows in TTEdit mode
    bool sshight{false}, mrs_high{false}, mrs_low{false}, mass_high{false}, bf_high{false}, bf_low{false}, pwr_high{false}, sig_s_high{false}, sig_s_Low{false};
///<Message flags in TT checks to stop being given twice

    double mtbf_hours{0};
///<Mean time between train failures in timetable clock hours
    float not_started_train_mins_late{0};
///< total late minutes of trains that haven't started yet on exit operation for locations not reached yet
    float operating_train_late_mins{0};
///< total late minutes of operating trains on exit operation for locations not reached yet
    float excess_lc_down_mins{0};
///< total excess time in minutes over the 3 minutes barriers down allowance for level crossings
    float tot_early_arr_mins{0};
///< values for performance file summary
    float tot_early_pass_mins{0};
    float tot_early_exit_mins{0};
    float tot_late_arr_mins{0};
    float tot_late_dep_mins{0};
    float tot_late_pass_mins{0};
    float tot_late_exit_mins{0};

// values for performance file summary
    int crashed_trains{0};
    int derailments{0};
    int early_arrivals{0};
    int early_passes{0};
    int early_exits{0}; //added at v2.9.1
    int incorrect_exits{0};
    int late_arrivals{0};
    int late_deps{0};
    int late_passes{0};
    int late_exits{0}; //added at v2.9.1
    int missed_stops{0};
    int on_time_arrivals{0};
    int on_time_deps{0};
    int on_time_passes{0};
    int on_time_exits{0}; //added at v2.9.1
    int other_missed_events{0};
    int skipped_ttb_events{0}; //added at v2.11.0
    int spad_events{0};
    int spad_risks{0};
    int tot_arr_dep_pass{0};
    int unexpectdd_exits{0};
    int num_failures{0}; // counts number of failed trains, added at v2.4.0
///< all these set to 0 in constructor
    int operating_train_late_arr{0};
///< total number of arrivals & departures for operating trains locations not reached yet (changed from OperatingTrainArrDep after 2.7.0 as departures now omitted)
    int not_started_train_late_arr{0};
///< total number of arrivals & departures for trains that haven't started yet for locations not reached yet (changed from NotStartedTrainArrDep after 2.7.0 as departures now omitted)
    int last_train_loaded{0};
///<displays last train loaded from session file, used for debugging
    int average_hours_int_val{0};
///<Input in MTBFEditBox in timetable hours, min value is 1 and max is 10,000. Here because performance file needs access
    ServiceCallingLocsList two_location_list; //added at v2.9.1
///<List of all ServiceRefs that have two or more same locations without a cdt between - loaded during SecondPassActions & displayed in Interface
    unsigned int op_time_to_act_update_counter;
///<new v2.2.0, incremented in Interface.cpp, controls updating for OpTimeToActPanel
    unsigned int op_action_panel_hint_delay_counter;
///<new v2.2.0 on start operation delays the op action panel headcode display for about 3 secs while hints shown
    unsigned int random_failure_counter;
///< new at v2.4.0 for train failures, resets after 53 seconds (53 prime so can trigger at any clock time)
    ContinuationAutoSigVector continuation_auto_sig;
///< vector for TContinuationAutoSigEntry objects
    ContinuationTrainExpectationMultiMap continuation_train_expectation_mm;
///< Multimap for TContinuationTrainExpectationEntry objects, the access key is the expectation time
    OpTimeToActMultiMap op_time_to_act_mm;
///<added v2.2.0 for Op time to act display
    OpTimeToActMultiMapIterator op_time_to_act_mm_iter;
///<added v2.2.0 for Op time to act display
    Actions::TrainDataVector train_data;
///< vector containing the internal timetable
    TrainVector trains;
///< vector containing all trains currently in the railway

//inline function

    bool LocServiceTimesLocationSort(LocServiceTimes i, LocServiceTimes j)
    {
        return (i.location < j.location);
    }

    bool LocServiceTimesArrTimeSort(LocServiceTimes i, LocServiceTimes j)
    {
        return (i.arr_time < j.arr_time);
    }

    bool LocServiceTimesDepTimeSort(LocServiceTimes i, LocServiceTimes j)
    {
        return (i.dep_time < j.dep_time);
    }
    bool LocServiceTimesAtLocTimeSort(LocServiceTimes i, LocServiceTimes j)
    {
        return (i.at_loc_time < j.at_loc_time);
    }

// functions defined in .cpp file

/// Removes duplicates from and sorts ServiceAndRepeatNumTotal into alphabetical order for arrivals (bool Arrival == true) & departures, also returns NumTrainsAtLoc after consolidation, used on creating the timetable conflict analysis file, Location needed to determine direction information
    std::string consolidateSARNTArrDep(int caller, const std::string &input, int &num_trains_at_loc, std::string &location, bool arrival, bool &analysis_error, int &max_number_of_same_directions);
/// Removes duplicates from and sorts ServiceAndRepeatNumTotal into alphabetical order for AtLoc listing (similar to ArrDep but doesn't include times in the input & don't need Location), also returns NumTrainsAtLoc after consolidation, used on creating the timetable conflict analysis file
    std::string consolidateSARNTAtLoc(int caller, const std::string &input, int &num_trains_at_loc);
/// Similar to TTrain::GetNewServiceDepartureInfo for use in ContinuationEntryFloatingTTString
    std::string controllerGetNewServiceDepartureInfo(int caller, Actions::VectorIterator ptr, int repeat_num, std::shared_ptr<Actions::TrainDataEntry> tde_ptr, std::shared_ptr<Actions::TrainDataEntry> linked_train, int incremental_mins, std::string &ret_str);
/// Build string for use in floating window for expected trains at continuations
    std::string continuationEntryFloatingTTString(int caller, std::shared_ptr<Actions::TrainDataEntry> tt_dep, int repeat_num, int incremental_mins, int incremental_digits);
/// Check all timetable names in ExitList, if all same return " at [name]" + AllowableExits = elements, else just return "" & AllowableExits = elements.  Used in floating label for Next action and in formatted timetables.
    std::string getExitLocationAndAt(int caller, Containers::NumList &exit_list, std::string &allowed_exit) const;
/// Return the service headcode for the repeat service
    std::string getRepeatHeadCode(int caller, std::string base_head_code, int repeat_number, int inc_digits);
/// Introduce a new train to the railway, with the characteristics specified, returns true for success, if fails the EventType indicates the reason
    bool addTrain(int caller, int rear_position, int front_position, std::string head_code, int start_speed, int mass, double max_running_speed, double MaxBrakeRate,
                  double power_at_rail, std::string mode_str, std::shared_ptr<Actions::TrainDataEntry> train_data, int repear_number, int incremental_mins, int incremental_digits,
                  int signaller_speed, bool signaller_control, std::shared_ptr<Actions::EventType> event_type);
/// A shorthand function that returns true if the successor to a given timetable action command should be (or could be) an 'at location' command, used in timetable validation
    bool atLocSuccessor(const std::shared_ptr<Actions::VectorEntry> &av_entry);
/// Used to compile ExitList from a string list of element IDs, returns true for success or gives a message & returns false for failure, used in timetable validation
    bool checkAndPopulateListOfIDs(int caller, std::string id_set, Containers::NumList &exit_list, bool give_message);
/// A timetable validation function where all service cross references are checked for validity and set pointers and train information, return true for success
    bool checkCrossReferencesAndSetData(int caller, std::string &sought_head_code, const std::string &seeking_head_code, bool shuttle, bool give_messages);
/// A timetable validation function where referenced services are checked for uniqueness, returns true for success
    bool checkForDuplicateCrossReferences(int caller, std::string &main_head_code, const std::string &second_head_code, bool give_messages);
/// Returns true if the headcode complies with requirements
    bool checkHeadCodeValidity(int caller, bool give_messages, std::string &head_code);
/// Returns true if the location name complies with requirements
    bool checkLocationValidity(int caller, std::string &loc_str, bool give_messages, bool check_locations_exist_in_railway);
/// A timetable validation function where cross references are checked for validity for non-repeating shuttle links, pointers and train information are set, returns true for success
    bool checkNonRepeatingShuttleLinksAndSetData(int caller, std::string &main_head_code, std::string &non_repeating_head_code, bool give_messages);
/// The forward train is the finish shuttle entry 'Fns-sh', the reverse (new non-repeating service) time must = Forward time + (RepeatMins * RepeatNumber), return true for success
    bool checkNonRepeatingShuttleLinkTime(int caller, boost::posix_time::time_duration &reverse_event_time, boost::posix_time::time_duration &forward_event_time, int repeat_mins, int repeat_number);
/// Part of the session file integrity check for ContinuationAutoSigEntries, true for success
    bool checkSessionContinuationAutoSigEntries(int caller, std::ifstream &session_file);
/// Part of the session file integrity check for locked routes, true for success
    bool checkSessionLockedRoutes(int caller, std::ifstream &session_file);
/// Part of the session file integrity check for train entries, true for success
    bool checkSessionTrains(int caller, std::ifstream &in_file);
/// Check that shuttle link services have consistent times, true for success
    bool checkShuttleRepeatTime(int caller, boost::posix_time::time_duration &forward_event_time, boost::posix_time::time_duration &reverse_event_time, int repeart_minutes);
/// Check that each shuttle service ends either in Fns or Fxx-sh (though a single service can't end in Fxx-sh), and that when the Fxx-sh is reached it references the original start and not another shuttle - not allowed to link two shuttles, true for success
    bool checkShuttleServiceIntegrity(int caller, std::shared_ptr<Actions::TrainDataEntry> td_entry, bool give_messages);
/// Called when trying to introduce a new train - checks for points in correct orientation, that no train is already at the train starting position, and not starting on a locked route
    bool checkStartAllowable(int caller, int rear_position, int rear_exit_pos, std::string &head_code, bool ReportFlag, std::shared_ptr<Actions::EventType> &event_type);
/// A timetable validation function where train starting positions are checked for validity, true for success
    bool checkStartPositionValidity(int caller, std::string &rear_element_str, std::string &front_element_str, bool give_message);
/// returns true if the time complies with requirements
    bool checkTimeValidity(int Caller, std::string &time_str, boost::posix_time::time_duration &time);
/// Generate a timetable analysis file in the 'Formatted Timetables' folder, return false if failed for any reason
    bool createTTAnalysisFile(int caller, std::string &railway_title, std::string &timetable_title, std::string &cur_dir, bool arr_checked, bool dep_checked, bool at_loc_chcked, bool dir_checked, int arr_range, int dep_range);
/// New trains introduced with 'Snt' may be at a timetabled location or elsewhere. This function checks and returns true for at a timetabled location.
    bool isSNTEntryLocated(int caller, const std::shared_ptr<Actions::TrainDataEntry> &td_entry, std::string &location_name);
/// Checks the last two characters in HeadCode and returns true if both are digits
    bool last2CharactersBothDigits(int caller, std::string &head_code);
/// A shorthand function that returns true if the successor to a given timetable action command should be (or could be) an en-route command, used in timetable validation
    bool movingSuccessor(const std::shared_ptr<Actions::VectorEntry> &av_entry);
/// Carry out preliminary (mainly syntax) validity checks on a single timetable service entry and (if FinalCall true) set the internal timetable data values, return true for success
    bool processOneTimetableLine(int caller, int count, std::string &one_line, bool &end_of_file, bool final_call, bool give_messages, bool check_locations_exist_in_railway);
/// Determines whether two services are running in the same direction when they arrive or depart from Location
    bool sameDirection(int caller, std::string &ref_1, std::string &ref_2, std::string &time_1, std::string &time_2, int repeat_num_1, int repeat_num_2, ServiceCallingLocsList list_1,
                       ServiceCallingLocsList list_2, std::string &location, bool arrival);
/// Carry out further detailed timetable consistency checks, return true for success
    bool secondPassActions(int caller, bool give_messages, bool &two_location); //TwoLocationFlag added at v2.9.1
/// Parse a single timetable service action, return true for success
    bool splitEntry(int caller, std::string &one_entry, bool give_messages, bool check_locations_exist_railway, std::string &first, std::string &second,
                    std::string &third, std::string &fourth, int &rear_start_or_repeat_mins, int &front_start_position, Timetable::FormatType &ttb_fmt_type,
                    Timetable::LocationType &location_type, Timetable::SequenceType &sequence_type, Timetable::ShuttleLinkType &shuttle_link_type, Containers::NumList &exit_list,
                    bool &warning);
/// Parse a timetable repeat entry, return true for success
    bool splitRepeat(int caller, std::string &one_entry, int &rear_start_or_repeat_mins, int &front_start_or_repeat_digits, int &repeat_number, bool give_messages);
/// Parse a train information entry, return true for success;  PowerAtRail changed to double& from int& at v2.4.0
    bool splitTrainInfo(int caller, std::string &train_info_str, std::string &head_code, std::string &description, int &start_speed, int &max_running_speed, int &mass,
                        double &max_brake_rate, double &power_at_rail, int &signaller_speed, bool give_messages);
/// Checks overall timetable integrity, calls many other specific checking functions, returns true for success
    bool timetableIntegrityCheck(int caller, char *file_name, bool give_messages, bool check_locations_exist_railway);
/// new at v2.4.0 return true if find the train (added at v2.4.0 as can select a removed train in ActionsDueListBox before it updates see LiWinDom error report via Discord on 23/04/20
    bool trainExistsAtIdent(int caller, int train_id);
/// check whether the two times are within the range in minutes specified and return true if so. For an equal time check MinuteRange = 0;
    bool withinTimeRange(int caller, std::string time_1, std::string &time_2, int minute_range);
/// new v2.2.0 (DistanceToExit added for multiplayer), calcs distances to red signal & exit, returns -1 for no signal found, for autosigs route after next red signal & other conditions, also totals up location stop times before the red signal and returns value in StopTime
    int calcDistanceToRedSignalandStopTime(int caller, int track_vector_position, int track_vector_position_entry_pos, bool sig_control_and_can_pass_red_signal,
                                           std::shared_ptr<Actions::VectorEntry> av_ptr, std::string &head_code, int train_id, float &current_stop_time, float &later_stop_time,
                                           float &recoverable_time, int &av_track_speed, int &distance_to_exit, Containers::HVShortPair &exit_pair);
/// Return the track entry link (Link[]) array position for the given train on track element at track vector position TrackVectorNumber
    int entryPos(int caller, int train_id_in, int track_vector_number);
/// Get the interval between repeats
    boost::posix_time::time_duration getControllerTrainTime(int caller, boost::posix_time::time_duration &time, int repeat_number, int incremental_mins);
/// Return the repeating service time
    boost::posix_time::time_duration getRepeatTime(int caller, boost::posix_time::time_duration &basic_time, int RepeatNumber, int IncMinutes);
/// Return a reference to the train at position VecPos in the TrainVector, carries out range checking on VecPos
    Train &trainVectorAt(int caller, int vec_pos);
/// Map of times to exit & exit coordinates
    Containers::TimeToExitMultiMap time_to_exit_mm;
/// Return a reference to the train with ID TrainID, carries out validity checking on TrainID
    Train &trainVectorAtIdent(int caller, int train_id);
////Return the TrainDataVector entry corresponding to ServiceReference, FinishType is 0 for end of service or 1 for a follow-on service
    Actions::TrainDataEntry getServiceFromVector(std::string &caller, std::string &service_reference, std::shared_ptr<Actions::TrainDataVector> vector, bool &finish_type, bool &found_flag);
/// populate the ContinuationTrainExpectationMultiMap during timetable loading
    void buildContinuationTrainExpectationMultiMap(int caller);
/// calculates additional lateness values for trains that haven't reached their destinations yet
    void calcOperatingAndNotStartedTrainLateness(int caller);
/// Examines the internal timetable (TrainDataVector) and creates from it a chronological (.txt) timetable and also a traditional spreadsheet format (.csv) timetable
    void createFormattedTimetable(int caller, const std::string &railway_title, const std::string &timetable_title, const std::string &cur_dir);
/// called when exiting operation mode to delete all trains and timetable data etc
    void finishedOperation(int caller);
/// load ContinuationAutoSigEntries from a session file
    void loadSessionContinuationAutoSigEntries(int caller, std::ifstream &session_file);
/// load locked routes from a session file
    void loadSessionLockedRoutes(int caller, std::ifstream &session_file);
/// load trains from a session file
    void loadSessionTrains(int caller, std::ifstream &session_file);
/// Send an error message to the performance log and file, and as a warning if appropriate
    void logActionError(int caller, const std::string &head_code, const std::string &other_head_code, Actions::EventType action_evnt_typ, const std::string location_id);
/// store Str to the event log - moved from TUtilities for v0.6 so can record the tt clock value
    void logEvent(std::string str);
/// called every clock tick to introduce new trains and update existing trains
    void operate(int Caller);
/// Plots all trains on screen in zoomed-out mode, state of 'Flash' determines whether the flashing trains are plotted or not
    void plotAllTrainsInZoomOutMode(int caller, bool flash);
/// new v2.2.0 for OperatorActionPanel (OperatorActionPanel changed for ActionsDueForm at v2.13.0)
    void rebuildOpTimeToActMultimap(int caller);
/// new for multiplayer
    void rebuildTimeToExitMultiMap(int caller);
/// plot all trains on the display
    // void replotTrains(int caller, TDisplay *Disp);
/// save ContinuationAutoSigEntries to a session file
    void saveSessionContinuationAutoSigEntries(int caller, std::ofstream &session_file);
/// save locked routes to a session file
    void saveSessionLockedRoutes(int caller, std::ofstream &session_file);
/// save trains to a session file
    void saveSessionTrains(int caller, std::ofstream &session_file);
/// diagnostic function to store all train data to a file for examination, not used normally
    void saveTrainDataVectorToFile(int caller);
/// Give a user message during timetable integrity checking if GiveMessages is true, ignore if false
    void secondPassMessage(bool give_messages, const std::string &message);
/// At the end of operation a summary of overall performance is sent to the performance file by this function
    void sendPerformanceSummary(int caller, std::ofstream &per_file);
/// This sets all the warning flags (CrashWarning, DerailWarning etc) to their required states after a session file has been loaded
    void setWarningFlags(int caller);
/// Outputs the single service vector for train direction analysis purposes in timetable conflict analysis
    void singleServiceOutput(int caller, int ss_vector_number, Containers::NumList marker_list, Actions::TrainDataVector &single_service_vec, std::ofstream &vec_file);
/// sends a message to the user and stops the timetable clock while it is displayed
    void stopTTClockMessage(int caller, const std::string &message);
/// change an extended headcode to an ordinary 4 character headcode
    void stripExcessFromHeadCode(int caller, const std::string &head_code);
/// Strip both leading and trailing spaces at ends of Input and spaces before and after all commas and semicolons within Input, used to rationalise timetable service entries for internal use
    void stripSpaces(int caller, const std::string &input);
/// Sends a message to the user if GiveMessages is true, including ServiceReference (see above) if not null, used for timetable error messages
    void timetableMessage(bool give_messages, const std::string &message);
/// unplot all trains from screen
    void UnplotTrains(int Caller);
/// Called by TInterface::SaveOperatingImage1Click) to write all trains to the image file
    //void WriteTrainsToImage(int Caller, Graphics::TBitmap *Bitmap);

/// Destructor
    ~TrainController();
};
};