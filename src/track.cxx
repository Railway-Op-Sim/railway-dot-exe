#include "railos/track.hxx"
#include "railos/logging.hxx"
#include "railos/utilities.hxx"
#include <stdexcept>
#include <string>

namespace RailOS::Track {

bool route_flash_flag{false};
bool point_flash_flag{false};

bool TrackElement::operator!=(TrackElement rh_element) {
  if ((this->h_loc != rh_element.h_loc) || (this->v_loc != rh_element.v_loc) ||
      (this->speed_tag != rh_element.speed_tag)) {
    return (true);
  } else {
    return (false);
  }
}

// void TrackElement::plotVariableTrackElement(int caller, TDisplay *Disp) const
// // 'Variable' in the sense that element might be striped or non-striped
// {
//     Utilities->CallLog.push_back(Utilities->TimeStamp() + "," + AnsiString(Caller) + ",PlotVariableTrackElement");
//     Graphics::TBitmap *GraphicOutput = GraphicPtr;

//     if(LocationName == "")
//     {
//         switch(SpeedTag)
//         {
//         case 76: // t platform
//             GraphicOutput = RailGraphics->gl76Striped;
//             break;

//         case 77: // h platform
//             GraphicOutput = RailGraphics->bm77Striped;
//             break;

//         case 78: // v platform
//             GraphicOutput = RailGraphics->bm78Striped;
//             break;

//         case 79: // r platform
//             GraphicOutput = RailGraphics->gl79Striped;
//             break;

//         case 96: // concourse
//             GraphicOutput = RailGraphics->ConcourseStriped;
//             break;

//         case 129: // v footbridge
//             GraphicOutput = RailGraphics->gl129Striped;
//             break;

//         case 130: // h footbridge
//             GraphicOutput = RailGraphics->gl130Striped;
//             break;

//         case 131: // non-station named loc
//             GraphicOutput = RailGraphics->bmNameStriped;
//             break;

//         case 145: // v underpass
//             GraphicOutput = RailGraphics->gl145Striped;
//             break;

//         case 146: // h underpass
//             GraphicOutput = RailGraphics->gl146Striped;
//             break;

//         default:
//             GraphicOutput = GraphicPtr;
//             break;
//         }
//     }
//     Disp->PlotOutput(34, HLoc * 16, VLoc * 16, GraphicOutput);
//     //deal with TSRs
//     if((Type == Simple) && Failed) //added at v2.13.0
//     {
//        Disp->GetImage()->Canvas->Draw((HLoc - Display->DisplayOffsetH) * 16, (VLoc - Display->DisplayOffsetV) * 16, RailGraphics->BlackOctagon); //indicates that it has failed
//     }
//     Utilities->CallLogPop(1332);
// }

std::string TrackElement::logTrack(int caller) const
// for debugging when passes as a call parameter
{
    return "TrkEl:-," + element_id + "," + location_name + "," + std::to_string(train_id_on_element) + "," +
        std::to_string(train_id_on_bridge_or_failed_point_orig_speed_limit_01) + "," + std::to_string(train_id_on_bridge_or_failed_point_orig_speed_limit_23) + ",EndTrkEl,";
}

TrackElement::TrackElement(FixedTrackPiece input) : FixedTrackPiece(input), attribute(0), calling_on_set(false), length_01(Utilities::default_track_length), length_23(-1), speed_limit_01(Utilities::default_track_speed_limit), speed_limit_23(-1),
    train_id_on_element(-1), train_id_on_bridge_or_failed_point_orig_speed_limit_01(-1), train_id_on_bridge_or_failed_point_orig_speed_limit_23(-1), station_entry_stop_link_pos_1(-1), station_entry_stop_link_pos_2(-1),
    sig_aspect(FourAspect)
{
    if((track_type == Type::Points) || (track_type == Type::Crossover) || (track_type == Type::Bridge))
    {
        length_23 = Utilities::default_track_length;
        speed_limit_23 = Utilities::default_track_speed_limit;
    }
}

bool MapComp:: operator()(const HVPair& lower, const HVPair& higher) const // HLoc  VLoc
{
    if(lower.second < higher.second)
    {
        return true;
    }
    else if(lower.second > higher.second)
    {
        return false;
    }
    else if(lower.second == higher.second)
    {
        if(lower.first < higher.first)
        {
            return true;
        }
    }
    return false;
}

PrefDirElement::PrefDirElement(TrackElement element_in, int e_link_in, int e_link_pos_in, int x_link_in, int x_link_pos_in, int track_vector_pos_in)
    : TrackElement(element_in), e_link(e_link_in), e_link_pos(e_link_pos_in), x_link(x_link_in), x_link_pos(x_link_pos_in), track_vector_pos(track_vector_pos_in),
    check_count(9), is_a_route(false), auto_signals(false), pref_dir_route(false)
{
    if(!entryExitNumber())
    {
        throw std::runtime_error("EXNumber failure in TPrefDirElement constructor");
    }
    // EXGraphicPtr = GetPrefDirGraphicPtr();
    // EntryDirectionGraphicPtr = GetDirectionPrefDirGraphicPtr();
}

std::string PrefDirElement::logPrefDir() const
// for debugging when passed as a call parameter
{
    return "PthEl:-," + std::to_string(e_link) + "," + std::to_string(e_link_pos) + "," + std::to_string(x_link) + "," + std::to_string(x_link_pos) + "," +
        std::to_string(ex_number) + "," + std::to_string(track_vector_pos) + "," + std::to_string(static_cast<int>(auto_signals)) + "," + std::to_string(static_cast<int>(pref_dir_route)) +
        ",ElementID," + element_id + "," + location_name + "," + std::to_string(train_id_on_element) + "," + std::to_string(train_id_on_bridge_or_failed_point_orig_speed_limit_01) + "," +
        std::to_string(train_id_on_bridge_or_failed_point_orig_speed_limit_23);
}

std::shared_ptr<Track> track{new Track()};

// ---------------------------------------------------------------------------

bool PrefDirElement::entryExitNumber() // true for valid number
/*
          Computes a number corresponding to ELink & Xlink if set, or to the entry and exit link values for the track
          at Link[0] and Link[1], or, if ELink or XLink not set, and a complex (4-entry) element, return false for error message.
          This should be OK because only elements for which ELink & XLink not set are PrefDir/route start elements and leading points
          as temporary end of PrefDir, and in both cases this function is not called as the direction is not displayed for these elements.
          Uses simple links between any 2 entry & exit points for use in displaying PrefDir or route graphics, or original graphic during
          route flashing.  Should only be called when ELink & XLink set, or when ELinkPos & XLinkPos set deliberately from a
          TTrackElement during route setting functions.  If a bridge then an additional check is made in case the graphic needed
          corresponds to an undebridge, i.e a gap needed between entry and exit.  In this case the EXNumber is increased by
          16 so as to be unique.  Returns true if valid and sets EXNumber to the selected value.
*/

{
    Logging::call_log.push_back(Utilities::getTimeStamp() + ",EntryExitNumber");
    int ex_array_[16][2]{
        {4, 6}, {2, 8}, // horizontal & vertical
        {2, 4}, {6, 2}, {8, 6}, {4, 8},    // sharp curves
        {1, 6}, {3, 8}, {9, 4}, {7, 2}, {1, 8}, {3, 4}, {9, 2}, {7, 6},    // loose curves
        {1, 9}, {3, 7}};    // forward & reverse diagonals

    int ex_num_{-1};
    int entry_{0}, exit_{0};

    if(e_link > -1)
    {
        entry_ = e_link; // pick up simple elements even if ELink &/or XLink not set, as no ambiguity
    }
    else if(link[2] == -1)
    {
        entry_ = link[0];
    }
    else
    {
        Logging::popCallLog(122);
        return false;
    }
    if(x_link > -1)
    {
        exit_ = x_link;
    }
    else if(link[2] == -1)
    {
        exit_ = link[1];
    }
    else
    {
        Logging::popCallLog(123);
        return false;
    }
    for(int x{0}; x < 16; x++)
    {
        if(((entry_ == ex_array_[x][0]) && (exit_ == ex_array_[x][1])) || ((entry_ == ex_array_[x][1]) && (exit_ == ex_array_[x][0]))) //added extra brackets round && segments at v2.9.1
        {
            ex_num_ = x;
        }
    }
    if(ex_num_ == -1)
    {
        Logging::popCallLog(124);
        return(false);
    }
    int br_num_{-1};

/* The order for bridge entries & exits is as below.  Note that there are 3 of each type,
       the graphic for each of which is different because of the shape of the overbridge.  The basic
       entry/exit value is computed above, and this used to select only from elements with that entry/exit
       value that is an underbridge, i.e overbridges ignored as the normal graphic is OK for them.
       int BrEXArray[24][2] = {
       {4,6},{2,8},{1,9},{3,7},
       {1,9},{3,7},{1,9},{3,7},
       {2,8},{4,6},{2,8},{4,6}
*/

    if(track_type == Type::Bridge)
    {
        switch(ex_num_) {
            case 1:
                switch(speed_tag) {
                    case 49:
                        br_num_ = 1 + 16;
                        break;
                    case 54:
                        br_num_ = 8 + 16;
                        break;
                    case 55:
                        br_num_ = 10 + 16;
                        break;
                    default:
                        break;
                }
                break;
            case 0:
                switch(speed_tag) {
                    case 48:
                        br_num_ = 0 +16;
                        break;
                    case 58:
                        br_num_ = 11 + 16;
                        break;
                    case 59:
                        br_num_ = 9 + 16;
                        break;
                    default:
                        break;
                }
            case 14:
                switch(speed_tag) {
                    case 50:
                        br_num_ = 2 + 16;
                        break;
                    case 52:
                        br_num_ = 5 + 16;
                        break;
                    case 57:
                        br_num_ = 6 + 16;
                        break;
                    default:
                        break;
                }
            case 15:
                switch(speed_tag) {
                    case 51:
                        br_num_ = 3 + 16;
                        break;
                    case 53:
                        br_num_ = 7 + 16;
                        break;
                    case 56:
                        br_num_ = 5 + 16;
                        break;
                    default:
                        break;
                }
        }
    }
    if(br_num_ == -1)
    {
        ex_number = ex_num_;
    }
    else
    {
        ex_number = br_num_;
    }
    Logging::popCallLog(125);
    return true;
}

bool Track::noActiveOrInactiveTrack(int caller)
{
    Logging::call_log.push_back(Utilities::getTimeStamp() + "," + std::to_string(caller) + ",NoTrack");
    bool track_present_{false};

    if(inactive_tracks.size() != 0)
    {
        Logging::popCallLog(1333);
        return false;
    }
    else if(tracks.size() == 0)
    {
        Logging::popCallLog(1334);
        return true;
    }
    else
    {
        for(unsigned int x{0}; x < tracks.size(); x++)
        {
            if((trackElementAt(1042, x).speed_tag != 0))
            {
                track_present_ = true;
            }
        }
    }
    Logging::popCallLog(1335);
    return(!track_present_);
}

// ---------------------------------------------------------------------------

bool Track::noActiveTrack(int caller)
{
    Logging::call_log.push_back(Utilities::getTimeStamp() + "," + std::to_string(caller) + ",NoActiveTrack");
    bool track_present_{false};

    if(tracks.size() == 0)
    {
        Logging::popCallLog(1582);
        return true;
    }
    else
    {
        for(unsigned int x{0}; x < tracks.size(); x++)
        {
            if((trackElementAt(1043, x).speed_tag != 0))
            {
                track_present_ = true;
            }
            break;
        }
    }
    Logging::popCallLog(1583);
    return(!track_present_);
}

// ---------------------------------------------------------------------------

void Track::plotAndAddTrackElement(int caller, int current_tag, int aspect, int h_loc_input, int v_loc_input, bool &track_linking_required_flag, bool internal_checks)
// TrackLinkingRequiredFlag only relates to elements that require track linking after plotting - used to set TrackFinished
// to false in calling function. New at v2.2.0 new parameter 'Aspect' to ensure signals plotted with correct number of aspects (for pasting)
// and also when zero and combined with SignalPost to indicate that adding track rather than pasting
{
    Logging::call_log.push_back(Utilities::getTimeStamp() + "," + std::to_string(caller) + ",PlotAndAddTrackElement," + std::to_string(current_tag) + "," +
                                 std::to_string(h_loc_input) + "," + std::to_string(v_loc_input) + "," + std::to_string(static_cast<int>(internal_checks)));
    
    bool plat_allowed_flag_{false};

    track_linking_required_flag = false;
/*
      Not erase, that covered separately.
      First check if Current SpeedButton assigned, then check if a platform and only
      permit if an appropriate trackpiece already there & not a same platform there.
      - can't enter a platform without track first.
      Then for non-platforms, check if a track piece already present at location &
      reject if so.
*/

    LocationNameMultiMapEntry location_name_entry_;

    location_name_entry_.first = "";

    if(current_tag == 0)
    {
        Logging::popCallLog(429);
        return; // not assigned yet
    }

    TrackElement temp_track_element_(fixed_track_array.fixed_track_piece[current_tag]);

    temp_track_element_.h_loc = h_loc_input;
    temp_track_element_.v_loc = v_loc_input;

    setElementID(1, temp_track_element_); // TempTrackElement is the one to be added
// new at version 0.6 - set signal aspect depending on build mode

    if(temp_track_element_.track_type == Type::SignalPost)
    {

        switch(aspect) {
            case 0:
                if(signal_aspect_build_mode == ThreeAspectBuild)
                {
                    temp_track_element_.sig_aspect = TrackElement::ThreeAspect;
                }
                else if(signal_aspect_build_mode == TwoAspectBuild)
                {
                    temp_track_element_.sig_aspect = TrackElement::TwoAspect;
                }
                else if(signal_aspect_build_mode == GroundSignalBuild)
                {
                    temp_track_element_.sig_aspect = TrackElement::GroundSignal;
                }
                else
                {
                    temp_track_element_.sig_aspect = TrackElement::FourAspect;
                }
                break;
            case 1:
                temp_track_element_.sig_aspect = TrackElement::GroundSignal;
                break;
            case 2:
                temp_track_element_.sig_aspect = TrackElement::TwoAspect;
                break;
            case 3:
                temp_track_element_.sig_aspect = TrackElement::ThreeAspect;
                break;
            default:
                temp_track_element_.sig_aspect = TrackElement::FourAspect;
        }
    }

    bool found_flag_{false}, inactive_found_flag_{false}, non_station_or_lc_present_{false}, platform_present_{false};

    int vec_pos_{getVectorPositionFromTrackMap(12, h_loc_input, v_loc_input, found_flag_)}; // active track already there

    IMPair im_pair_{getVectorPositionsFromInactiveTrackMap(5, h_loc_input, v_loc_input, inactive_found_flag_)}; // inactive track already there
    
    int inactive_speed_tag_1_{0}, inactive_speed_tag_2_{0};

    if(inactive_found_flag_) // check if a LocationName already there & if so disallow platform
    {
        if(inactiveTrackElementAt(4, im_pair_.first).track_type == Type::NamedNonStationLocation)
        {
            non_station_or_lc_present_ = true;
        }
        if(inactiveTrackElementAt(117, im_pair_.first).track_type == Type::LevelCrossing)
        {
            non_station_or_lc_present_ = true;
        }
        if(inactiveTrackElementAt(5, im_pair_.first).track_type == Type::Platform)
        {
            platform_present_ = true;
        }
        // no need to check IMPair.second since if that exists it is because .first is a platform
        inactive_speed_tag_1_ = inactiveTrackElementAt(6, im_pair_.first).speed_tag;
        inactive_speed_tag_2_ = inactiveTrackElementAt(7, im_pair_.second).speed_tag; // note .first & .second will be same if only one present
    }
// check platforms
    if(temp_track_element_.track_type == Type::Platform)
    {
        if(found_flag_) // active track element already there
        {
            if(inactive_found_flag_ && ((temp_track_element_.speed_tag == inactive_speed_tag_1_) || (temp_track_element_.speed_tag == inactive_speed_tag_2_)))
            {
                ;
            }
            // same platform type already there so above keeps PlatAllowedFlag false
            else if((temp_track_element_.speed_tag == 76) && (top_plat_allowed.find(trackElementAt(1044, vec_pos_).speed_tag) != top_plat_allowed.end()) && !non_station_or_lc_present_)
            // won't allow a same platform, as TopPlatAllowed not valid for a same platform <--NO, only checks active track, same plat disallowed by first line after if(FoundFlag)
            {
                plat_allowed_flag_ = true;
            }
            else if((temp_track_element_.speed_tag == 77) && (bot_plat_allowed.find(trackElementAt(vec_pos_, vec_pos_).speed_tag) != bot_plat_allowed.end()) && !non_station_or_lc_present_)
            {
                plat_allowed_flag_ = true;
            }
            else if((temp_track_element_.speed_tag == 78) && (left_plat_allowed.find(trackElementAt(1046, vec_pos_).speed_tag) != left_plat_allowed.end()) && !non_station_or_lc_present_)
            {
                plat_allowed_flag_ = true;
            }
            else if((temp_track_element_.speed_tag == 79) && (right_plat_allowed.find(trackElementAt(1047, vec_pos_).speed_tag) != right_plat_allowed.end()) && !non_station_or_lc_present_)
            {
                plat_allowed_flag_ = true;
            }

            if(plat_allowed_flag_)
            {
                track_linking_required_flag = true; // needed in order to call LinkTrack

                trackPush(1, temp_track_element_);

                searchForAndUpdateLocationName(1, temp_track_element_.h_loc, temp_track_element_.v_loc, temp_track_element_.speed_tag);
                // checks all adjacent locations and if any name found that one is used for all elements that are now linked to it
                // Must be called AFTER TrackPush
                // No need to plot the element - Clearand ... called after this function called
                // set corresponding track element length to 100m & give message if was different    drop in v2.4.0
                // note can only be Length01 since even if points then only the straight part can be adjacent to the platform
// drop in v2.4.0                if(TrackElementAt(2, VecPos).Length01 != DefaultTrackLength) ShowMessage("Note:  The track element at this location has a length of " +
// AnsiString(TrackElementAt(3, VecPos).Length01) + "m.  It will be reset to 100m since all platform track lengths are fixed at 100m");
// TrackElementAt(4, VecPos).Length01 = DefaultTrackLength;
                if(internal_checks)
                {
                    checkMapAndInactiveTrack(5); // test
                    checkLocationNameMultiMap(4); // test
                }
                Logging::popCallLog(430);
                return;
            }
        } // if(FoundFlag)

        Logging::popCallLog(431);
        return;
    } // if platform

// check if element is a LocationName - OK if placed on an allowable track element, or on a blank element
    if(temp_track_element_.track_type == Type::NamedNonStationLocation)
    {
        if((found_flag_ && (name_allowed.find(trackElementAt(1048, vec_pos_).speed_tag) != name_allowed.end()) && !platform_present_ && !inactive_found_flag_) ||
           (!found_flag_ && !inactive_found_flag_))
        // need to add && !NonStationOrLevelCrossingPresent, or better - !InactiveFoundFlag to above FoundFlag condition <-- OK done
        {
            track_linking_required_flag = true; // needed in case have named a continuation, need to check if adjacent element named
 
            trackPush(2, temp_track_element_);

            searchForAndUpdateLocationName(2, temp_track_element_.h_loc, temp_track_element_.v_loc, temp_track_element_.speed_tag);
            // checks all adjacent locations and if any name found that one is used for all elements that are now linked to it
            if(vec_pos_ > -1) // need to allow for non-station named locations that aren't on tracks
            {
// drop in v2.4.0                if(TrackElementAt(830, VecPos).Length01 != DefaultTrackLength) ShowMessage("Note:  The track element at this location has a length of " +
// AnsiString(TrackElementAt(831, VecPos).Length01) + "m.  It will be reset to 100m since all named location track lengths are fixed at 100m");
// TrackElementAt(832, VecPos).Length01 = DefaultTrackLength; //NB named locations can only be placed at one track elements
            }
            if(internal_checks)
            {
                checkMapAndInactiveTrack(11); // test
                checkLocationNameMultiMap(12); // test
            }
            Logging::popCallLog(432);
            return;
        }
        else
        {
            Logging::popCallLog(433);
            return;
        }
    }
// check if a level crossing - OK if placed on a plain straight track
    if(temp_track_element_.track_type == Type::LevelCrossing)
    {
        if(found_flag_ && (level_crossing_allowed.find(trackElementAt(1049, vec_pos_).speed_tag) != level_crossing_allowed.end()) && !platform_present_ && !inactive_found_flag_)
        {
            trackPush(11, temp_track_element_);
            //plotRaisedLinkedLevelCrossingBarriers(0, trackElementAt(1050, vec_pos_).speed_tag, temp_track_element_.h_loc, temp_track_element_.v_loc, display); //always plots red
// no need for reference to LC element as can't be open
            track_linking_required_flag = true;
            Logging::popCallLog(1907);
            return;
        }
        else
        {
            Logging::popCallLog(1906);
            return; // was a level crossing but can't place it for some reason
        }
    }

// check if another element already there
    else if(found_flag_ || inactive_found_flag_)
    {
        Logging::popCallLog(434);
        return; // something already there (active or inactive track)
    }
// add LocationName if a FixedNamedLocationElement by checking for any adjacent names, then give all linked named location
// elements the same name - in case had linked 2 separately named locations - all get the one name that it finds
// first from an adjacent element search, also non-named location elements at platform locations have timetable name set
// do this after pushed into vector so that can use EnterLocationName

    if(temp_track_element_.fixed_named_location_element) // concourse or footcrossing (platforms & named non-station locations already dealt with)
    {
        trackPush(3, temp_track_element_);
        searchForAndUpdateLocationName(3, temp_track_element_.h_loc, temp_track_element_.v_loc, temp_track_element_.speed_tag);
        // checks all adjacent locations and if any name found that one is used for all elements that are now linked to it
    }
    else if(temp_track_element_.track_type == Type::Points)
    {
        trackPush(4, temp_track_element_);
        bool both_point_fillets_{true};
        //plotPoints(6, temp_track_element_, display, both_point_fillets_);
    }
    else if(temp_track_element_.track_type == Type::SignalPost)
    {
        trackPush(10, temp_track_element_);
        //PlotSignal(12, temp_track_element_, display);
    }
    else
    {
        trackPush(5, temp_track_element_);
        //temp_track_element_.plotVariableTrackElement(1, Display); // all named locations already dealt with so no ambiguity between striped & non-striped
    }
    if((temp_track_element_.track_type != Type::Concourse) && (temp_track_element_.track_type != Type::Parapet))
    {
        track_linking_required_flag = true; // plats & NamedLocs aleady dealt with
    }

    if(internal_checks)
    {
        checkMapAndTrack(2); // test
        checkMapAndInactiveTrack(2); // test
        checkLocationNameMultiMap(5); // test
    }
    Logging::popCallLog(2062);
}


// ---------------------------------------------------------------------------

bool PrefDirElement:: operator == (PrefDirElement rh_element)
/*
      Set == operator when TrackVectorPosition, ELink & XLink all same
*/
{
    if((this->track_vector_pos == rh_element.track_vector_pos) && (this->e_link == rh_element.e_link) && (this->x_link == rh_element.x_link))
    {
        return true;
    }
    else
    {
        return false;
    }
}

// ---------------------------------------------------------------------------

bool PrefDirElement:: operator != (PrefDirElement rh_element)
/*
      Set != operator when any of TrackVectorPosition, ELink or XLink different
*/
{
    if((this->track_vector_pos == rh_element.track_vector_pos) && (this->e_link == rh_element.e_link) && (this->x_link == rh_element.x_link))
    {
        return false;
    }
    else
    {
        return true;
    }
}

Track::Track()
{
// CurrentSpeedButtonTag = 0; //not assigned yet

    route_fail_message = "Unable to set a route:" + nl + nl + "it may be unreachable, perhaps because of failed points; " + nl + nl +
        "reachable but with too many different directions leading away from the start point  - set some points on the route required; " + nl + nl +
        "blocked by a train, another route or a changing level crossing; " + nl + nl +
        "or invalid - possibly due to a preferred direction mismatch, or a missed signal in a blue route or green route restricted to consecutive signals.";

    // GapFlashGreen = new TGraphicElement;
    // GapFlashRed = new TGraphicElement;
    // GapFlashGreen->LoadOverlayGraphic(1, RailGraphics->bmGreenEllipse);
    // GapFlashRed->LoadOverlayGraphic(2, RailGraphics->bmRedEllipse);

// Platform and default track element values
    top_plat_allowed = {1, 9, 10, 30, 31, 60, 61, 68, 69, 77, 125, 126, 129, 145};
// top & bot sigs, straights, straight points, buffers, signal, vert footcrossing, bot plat
    bot_plat_allowed = {1, 7, 8, 28, 29, 60, 61, 68, 69, 76, 125, 126, 129, 145};
    left_plat_allowed = {2, 12, 14, 33, 35, 62, 63, 70, 71, 79, 127, 128, 130, 146};
    right_plat_allowed = {2, 11, 13, 32, 34, 62, 63, 70, 71, 78, 127, 128, 130, 146};
    name_allowed = {1, 2, 3, 4, 5, 6, 20, 21, 22, 23, 24, 25, 26, 27, // disallow diagonals, points, crossovers, bridges, gaps,
    60, 61, 62, 63 , 68 , 69 , 70 , 71, 80, 81, 82, 83, 125, 126, 127, 128};     // diag continuations, diag buffers, footcrossings (diagonals may be OK
    // but as can't link diagonal locations would need solid blocks to allow linkage & that would look untidy except for single
    // elements, & can always use straights so leave out.) Allow horiz & vert signals as from v2.6.0
    level_crossing_allowed = {1, 2}; // only allow on straight tracks without direction markers
// Note platforms not allowed at continuations, but named non-station locations OK, though not allowed in timetables

    track_finished = false;
// DistancesSet = false;

    // SigElement temp_sig_table_[40] = // original four aspect
    // {{68, 0, RailGraphics->gl68}, {69, 0, RailGraphics->gl69}, {70, 0, RailGraphics->gl70}, {71, 0, RailGraphics->gl71}, {72, 0, RailGraphics->gl72},
    //  {73, 0, RailGraphics->bm73}, {74, 0, RailGraphics->bm74}, {75, 0, RailGraphics->gl75},

    //  {68, 1, RailGraphics->bm68yellow}, {69, 1, RailGraphics->bm69yellow}, {70, 1, RailGraphics->bm70yellow}, {71, 1, RailGraphics->bm71yellow},
    //  {72, 1, RailGraphics->bm72yellow}, {73, 1, RailGraphics->bm73yellow}, {74, 1, RailGraphics->bm74yellow}, {75, 1, RailGraphics->bm75yellow},

    //  {68, 2, RailGraphics->bm68dblyellow}, {69, 2, RailGraphics->bm69dblyellow}, {70, 2, RailGraphics->bm70dblyellow}, {71, 2, RailGraphics->bm71dblyellow},
    //  {72, 2, RailGraphics->bm72dblyellow}, {73, 2, RailGraphics->bm73dblyellow}, {74, 2, RailGraphics->bm74dblyellow}, {75, 2, RailGraphics->bm75dblyellow},

    //  {68, 3, RailGraphics->bm68green}, {69, 3, RailGraphics->bm69green}, {70, 3, RailGraphics->bm70green}, {71, 3, RailGraphics->bm71green},
    //  {72, 3, RailGraphics->bm72green}, {73, 3, RailGraphics->bm73green}, {74, 3, RailGraphics->bm74green}, {75, 3, RailGraphics->bm75green},

    //  {68, 4, RailGraphics->gl68}, {69, 4, RailGraphics->gl69},    // Attr 4 disused but leave in case re-instate
    //  {70, 4, RailGraphics->gl70}, {71, 4, RailGraphics->gl71}, {72, 4, RailGraphics->gl72}, {73, 4, RailGraphics->bm73}, {74, 4, RailGraphics->bm74},
    //  {75, 4, RailGraphics->gl75}};

    // for(int x = 0; x < 40; x++)
    // {
    //     sig_table[x] = temp_sig_table[x];
    // }

    // TSigElement TempSigTableThreeAspect[40] =
    // {{68, 0, RailGraphics->gl68}, {69, 0, RailGraphics->gl69}, {70, 0, RailGraphics->gl70}, {71, 0, RailGraphics->gl71}, {72, 0, RailGraphics->gl72},
    //  {73, 0, RailGraphics->bm73}, {74, 0, RailGraphics->bm74}, {75, 0, RailGraphics->gl75},

    //  {68, 1, RailGraphics->bm68yellow}, {69, 1, RailGraphics->bm69yellow}, {70, 1, RailGraphics->bm70yellow}, {71, 1, RailGraphics->bm71yellow},
    //  {72, 1, RailGraphics->bm72yellow}, {73, 1, RailGraphics->bm73yellow}, {74, 1, RailGraphics->bm74yellow}, {75, 1, RailGraphics->bm75yellow},

    //  {68, 2, RailGraphics->bm68green}, {69, 2, RailGraphics->bm69green}, {70, 2, RailGraphics->bm70green}, {71, 2, RailGraphics->bm71green},
    //  {72, 2, RailGraphics->bm72green}, {73, 2, RailGraphics->bm73green}, {74, 2, RailGraphics->bm74green}, {75, 2, RailGraphics->bm75green},

    //  {68, 3, RailGraphics->bm68green}, {69, 3, RailGraphics->bm69green}, {70, 3, RailGraphics->bm70green}, {71, 3, RailGraphics->bm71green},
    //  {72, 3, RailGraphics->bm72green}, {73, 3, RailGraphics->bm73green}, {74, 3, RailGraphics->bm74green}, {75, 3, RailGraphics->bm75green},

    //  {68, 4, RailGraphics->gl68}, {69, 4, RailGraphics->gl69},    // Attr 4 disused but leave in case re-instate
    //  {70, 4, RailGraphics->gl70}, {71, 4, RailGraphics->gl71}, {72, 4, RailGraphics->gl72}, {73, 4, RailGraphics->bm73}, {74, 4, RailGraphics->bm74},
    //  {75, 4, RailGraphics->gl75}};

    // for(int x = 0; x < 40; x++)
    // {
    //     SigTableThreeAspect[x] = TempSigTableThreeAspect[x];
    // }

    // TSigElement TempSigTableTwoAspect[40] =
    // {{68, 0, RailGraphics->gl68}, {69, 0, RailGraphics->gl69}, {70, 0, RailGraphics->gl70}, {71, 0, RailGraphics->gl71}, {72, 0, RailGraphics->gl72},
    //  {73, 0, RailGraphics->bm73}, {74, 0, RailGraphics->bm74}, {75, 0, RailGraphics->gl75},

    //  {68, 1, RailGraphics->bm68green}, {69, 1, RailGraphics->bm69green}, {70, 1, RailGraphics->bm70green}, {71, 1, RailGraphics->bm71green},
    //  {72, 1, RailGraphics->bm72green}, {73, 1, RailGraphics->bm73green}, {74, 1, RailGraphics->bm74green}, {75, 1, RailGraphics->bm75green},

    //  {68, 2, RailGraphics->bm68green}, {69, 2, RailGraphics->bm69green}, {70, 2, RailGraphics->bm70green}, {71, 2, RailGraphics->bm71green},
    //  {72, 2, RailGraphics->bm72green}, {73, 2, RailGraphics->bm73green}, {74, 2, RailGraphics->bm74green}, {75, 2, RailGraphics->bm75green},

    //  {68, 3, RailGraphics->bm68green}, {69, 3, RailGraphics->bm69green}, {70, 3, RailGraphics->bm70green}, {71, 3, RailGraphics->bm71green},
    //  {72, 3, RailGraphics->bm72green}, {73, 3, RailGraphics->bm73green}, {74, 3, RailGraphics->bm74green}, {75, 3, RailGraphics->bm75green},

    //  {68, 4, RailGraphics->gl68}, {69, 4, RailGraphics->gl69},    // Attr 4 disused but leave in case re-instate
    //  {70, 4, RailGraphics->gl70}, {71, 4, RailGraphics->gl71}, {72, 4, RailGraphics->gl72}, {73, 4, RailGraphics->bm73}, {74, 4, RailGraphics->bm74},
    //  {75, 4, RailGraphics->gl75}};

    // for(int x = 0; x < 40; x++)
    // {
    //     SigTableTwoAspect[x] = TempSigTableTwoAspect[x];
    // }

    // TSigElement TempSigTableGroundSignal[40] =
    // {{68, 0, RailGraphics->bm68grounddblred}, {69, 0, RailGraphics->bm69grounddblred}, {70, 0, RailGraphics->bm70grounddblred},
    //  {71, 0, RailGraphics->bm71grounddblred}, {72, 0, RailGraphics->bm72grounddblred}, {73, 0, RailGraphics->bm73grounddblred},
    //  {74, 0, RailGraphics->bm74grounddblred}, {75, 0, RailGraphics->bm75grounddblred},

    //  {68, 1, RailGraphics->bm68grounddblwhite}, {69, 1, RailGraphics->bm69grounddblwhite}, {70, 1, RailGraphics->bm70grounddblwhite},
    //  {71, 1, RailGraphics->bm71grounddblwhite}, {72, 1, RailGraphics->bm72grounddblwhite}, {73, 1, RailGraphics->bm73grounddblwhite},
    //  {74, 1, RailGraphics->bm74grounddblwhite}, {75, 1, RailGraphics->bm75grounddblwhite},

    //  {68, 2, RailGraphics->bm68grounddblwhite}, {69, 2, RailGraphics->bm69grounddblwhite}, {70, 2, RailGraphics->bm70grounddblwhite},
    //  {71, 2, RailGraphics->bm71grounddblwhite}, {72, 2, RailGraphics->bm72grounddblwhite}, {73, 2, RailGraphics->bm73grounddblwhite},
    //  {74, 2, RailGraphics->bm74grounddblwhite}, {75, 2, RailGraphics->bm75grounddblwhite},

    //  {68, 3, RailGraphics->bm68grounddblwhite}, {69, 3, RailGraphics->bm69grounddblwhite}, {70, 3, RailGraphics->bm70grounddblwhite},
    //  {71, 3, RailGraphics->bm71grounddblwhite}, {72, 3, RailGraphics->bm72grounddblwhite}, {73, 3, RailGraphics->bm73grounddblwhite},
    //  {74, 3, RailGraphics->bm74grounddblwhite}, {75, 3, RailGraphics->bm75grounddblwhite},

    //  {68, 4, RailGraphics->bm68grounddblred}, {69, 4, RailGraphics->bm69grounddblred},    // Attr 4 disused but leave in case re-instate
    //  {70, 4, RailGraphics->bm70grounddblred}, {71, 4, RailGraphics->bm71grounddblred}, {72, 4, RailGraphics->bm72grounddblred},
    //  {73, 4, RailGraphics->bm73grounddblred}, {74, 4, RailGraphics->bm74grounddblred}, {75, 4, RailGraphics->bm75grounddblred}};

    // for(int x = 0; x < 40; x++)
    // {
    //     SigTableGroundSignal[x] = TempSigTableGroundSignal[x];
    // }

    // TSigElement TempFailedSigTable[8] = // added at v2.13.0
    // {{68, 0, RailGraphics->FSig68}, {69, 0, RailGraphics->FSig69}, {70, 0, RailGraphics->FSig70}, {71, 0, RailGraphics->FSig71}, {72, 0, RailGraphics->FSig72},
    //  {73, 0, RailGraphics->FSig73}, {74, 0, RailGraphics->FSig74}, {75, 0, RailGraphics->FSig75}};

    // for(int x = 0; x < 8; x++)
    // {
    //     FailedSigTable[x] = TempFailedSigTable[x];
    // }
}


// ---------------------------------------------------------------------------s
}; // namespace RailOS::Track