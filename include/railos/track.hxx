#pragma once

#include "boost/date_time/posix_time/posix_time_config.hpp"
#include <fstream>
#include <list>
#include <map>
#include <memory>
#include <utility>
#include <vector>
#include <set>
#include <filesystem>

#include "railos/utilities.hxx"
#include "railos/logging.hxx"

#define FirstUnusedSpeedTagNumber                                              \
  147 // defined value for use in array sizing etc

namespace RailOS::Track {
typedef std::pair<int, int> HVPair;
///< h_loc/v_loc position pair

class TrackElement;

class PrefDirElement;
///< forward declaration because needed in TTrack

typedef std::vector<PrefDirElement> PrefDirVector;
///< forward declaration because needed in TTrack

/// Map and multimap comparator based on horizontal & vertical position
class MapComp {
public:
  /// h_loc v_loc
  bool operator()(const HVPair &lower, const HVPair &higher) const;
};

enum class Type
///< describes the type of track element
{
  Simple,
  Crossover,
  Points,
  Buffers,
  Bridge,
  SignalPost,
  Continuation,
  Platform,
  GapJump,
  FootCrossing,
  Unused,
  Concourse,
  Parapet,
  NamedNonStationLocation,
  Erase,
  LevelCrossing
};
// FootCrossing covers footbridge & underpass/surface, 'unused' was marker the
// for old 'text' number, since disused Concourse, Parapet,
// NamedNonStationLocation, Platform & LevelCrossing are the 5 types of inactive
// element Erase was the default active element used for erasing, not used now
// (all data members unset)

enum Configuration
///< describes the type of track link. 'End' is used for both buffer stop and
///< continuation entry/exit positions
{
  NotSet,
  Connection,
  End,
  Gap,
  Lead,
  Trail,
  CrossConn,
  Under,
  Signal
};

// ---------------------------------------------------------------------------
// PrefDir & Route functions
// ---------------------------------------------------------------------------

enum class TruncateReturnType
///< a flag used during route truncation to indicate the nature of the selected element, it could be not in a route (notInRoute), in a route but invalid (InRouteFalse), or in a route and valid (InRouteTrue)
{
    NotInRoute, InRouteTrue, InRouteFalse
};

enum class PrefDirRoute
///< used in TOnePrefDir::PrefDirMarker to indicate whether the function is being called for a preferred direction (prefDirCall) or a route (RouteCall)
{
    PrefDirCall, RouteCall
};

// FIXED TRACK :-
// All basic track building blocks & methods

class FixedTrackPiece {
public: // everything uses these - should really have Gets & Sets but too many
        // to change now
  bool fixed_named_location_element{false};
  ///< true for an element that can be named (platforms, concourse,
  ///< footcrossings & non-station named loactions)
  int speed_tag{0};
  ///< The element identification number - corresponds to the relevant
  ///< SpeedButton->Tag
  int link[4]{-1, -1, -1, -1};
  ///< Track connection link values, max. of 4, unused = -1, top lh diag.=1,
  ///< top=2, top rh diag.=3, left=4, right=6, bottom lh diag.=7, bottom=8,
  ///< bottom rh diag.=9
  //     Graphics::TBitmap *GraphicPtr;
  // ///< the track bitmap for display on the zoomed-in railway
  //     Graphics::TBitmap *SmallGraphicPtr;
  ///< the track bitmap for display on the zoomed-out railway

  Configuration config[4]{NotSet, NotSet, NotSet, NotSet};
  ///< the type of link - see TConfiguration above

  Type track_type;
  ///< the type of track element

  /// Plot the element on the railway display at position h_loc_input & v_loc_input
  void plotFixedTrackElement(int caller, int h_loc_input,
                             int v_loc_input) const;
  /// Constructor for building TTrack.FixedTrackArray - see below
  // FixedTrackPiece(int speed_tag, Type track_type, int lk_val[4],
  // Configuration config[4], Graphics::TBitmap *GraphicPtrVal,
  //                  Graphics::TBitmap *SmallGraphicPtrVal);
};

// ---------------------------------------------------------------------------

// VARIABLE TRACK :-

// ---------------------------------------------------------------------------

/* Note:  Should probably have used different derived classes for the different
   track types, to avoid all having attributes & other specific data, but by the
   time this occurred to me as a good idea it seemed likely to be more difficult
      to backtrack than to put up with the extra & unused data.
 */

/// Basic track elements as implemented in the overall railway layout
class TrackElement : public FixedTrackPiece {
public: // everything uses these - should really have Gets & Sets but too many
        // to change now
  std::string active_track_element_name;
  ///< Location name used either in the timetable or for a continuation
  ///< (continuation names not used in timetable as trains can't stop there).
  ///< Only active track elements where there are platforms or non-station named
  ///< locations have ActiveTrackElementNames
  std::string element_id;
  ///< the element identifier based on position in the railway
  std::string location_name;
  ///< location name not used for timetabling, only for identification:
  ///< platforms, non-station named locations, concourses and footcrossings have
  ///< LocationNames

  bool calling_on_set{false};
  ///< Used for for signals only when a train is being called on - used to plot
  ///< the position lights
  bool lc_plotted{false};
  ///< Utility marker to avoid plotting every element of a multitrack LC during
  ///< ClearandRebuildRailway
  bool temp_track_marker_01{false}, temp_track_marker_23{false};
  ///< Utility markers for program use, not used from v2.12.0
  bool failed{false};
  ///< New parameter added at v2.13.0 for failed points & TSRs
  int attribute{0};
  ///< special variable used only for points, signals & level crossings, ignored
  ///< otherwise; points 0=set to go straight, 1=set to diverge, where both legs
  ///< diverge 0=set to left fork; signals:  0=red; 1=yellow; 2=double yellow; 3
  ///< = green; Level crossing: 0 = raised barriers = closed to trains; 1 =
  ///< lowered barriers = open to trains; 2 = changing state = closed to trains
  int conn[4]{-1, -1, -1, -1};
  ///< Connecting element position in TrackVector, set to -1 if no connecting
  ///< link or if track not linked
  int con_link_pos[4]{-1, -1, -1, -1};
  ///< Connecting element link position (i.e. array positions of the connecting
  ///< element links, in same order as Link[4])
  int h_loc{-2000000000}, v_loc{-2000000000};
  ///< The h & v locations in the railway (top lh corner of the first build
  ///< screen = 0,0)
  int length_01{Utilities::default_track_length}, length_23{-1}, speed_limit_01{-1}, speed_limit_23{-1};
  ///< Element lengths and speed limits, ...01 is for the track with link
  ///< positions [0] and [1], ...23 for [2] and [3], set to -1 if not used
  ///< (lengths in m & speed limits in km/h)
  int station_entry_stop_link_pos_1{0}, station_entry_stop_link_pos_2{0};
  ///< Used for track at platforms and non-station named locations to mark the
  ///< train front element stop position, there are two for the two directions
  ///< of travel, set to -1 if not used
  int train_id_on_element{-1},
      train_id_on_bridge_or_failed_point_orig_speed_limit_01{-1},
      train_id_on_bridge_or_failed_point_orig_speed_limit_23{-1};
  ///< Set to the TrainID value for a bridge when a train is present on the
  ///< element, bridges can have two trains present so the ...01 and ...23
  ///< values give the TrainIDs for track with link positions [0] & [1], and [2]
  ///< & [3] respectively, set to -1 if no train present For a failed point
  ///< store the original speedlimits, names changed at v2.13.0 to cater for
  ///< failed points
  enum
  ///< added at version 0.6
  {
    FourAspect,
    ThreeAspect,
    TwoAspect,
    GroundSignal
  } sig_aspect{FourAspect};

  // functions defined in .cpp file

  bool operator==(TrackElement rHElement);
  ///< equivalence operator
  bool operator!=(TrackElement rHElement);
  ///< non-equivalence operator
  std::string logTrack(int caller) const;
  ///< Used to log track parameters for call stack logging
//   void plotVariableTrackElement(int caller, TDisplay *Disp) const;
  ///< Plot the element on the display 'variable' indicates that the element may
  ///< be named and if so may be plotted striped or solid depending on whether
  ///< the name has been set

  /// Constructor for specific type of element. Use very high neg. numbers as
  /// 'unset' values for h_loc & v_loc initially as can go high negatively
  /// legitimately, build from existing TTrackPiece with default values for
  /// extra members
  TrackElement(FixedTrackPiece input);

  TrackElement();
};

// ---------------------------------------------------------------------------
// PrefDir & Route elements
// ---------------------------------------------------------------------------

/// Basic preferred direction or route element - track element with additional members
class PrefDirElement : public TrackElement
{
protected:

    int e_link, e_link_pos;
///< entry link number & array position
    int x_link, x_link_pos;
///< exit link number & array position
    int ex_number;
///< used to facilitate identification of the appropriate preferred direction or route graphic
    int track_vector_pos;
///< TrackVectorPosition of the corresponding track element
    int check_count;
///< internal check value used when building preferred directions
    // Graphics::TBitmap *EXGraphicPtr, *EntryDirectionGraphicPtr;
///< pointers to the appropriate entry/exit graphic, or direction marker graphic, for preferred directions and routes

    bool operator == (PrefDirElement rh_element);
///< equivalence operator
    bool operator != (PrefDirElement rh_element);
///< non-equivalence operator

public:

    friend class OnePrefDir;
    friend class OneRoute;
    friend class AllRoutes;

    bool is_a_route;
///< false for Pref Dir, true for route
    bool auto_signals;
///< marker within the route for an AutoSignal route element
    bool pref_dir_route;
///< marker within the route for preferred direction route element

// inline functions

/// Position check
    bool isPosition(int position) const
    {
        if(track_vector_pos == position)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

/// Returns SpeedTag  //added at v2.9.2 for clipboard storage
    int getSpeedTag() const
    {
        return speed_tag;
    }

/// Returns h_loc  //added at v2.9.0 for clipboard storage
    int geth_loc() const
    {
        return h_loc;
    }

/// Returns v_loc  //added at v2.9.0 for clipboard storage
    int getv_loc() const
    {
        return v_loc;
    }

/// Returns ELink
    int getELink() const
    {
        return e_link;
    }

/// Returns the ELink array position
    int getELinkPos() const
    {
        return e_link_pos;
    }

/// Returns XLink
    int getXLink() const
    {
        return x_link;
    }

/// Returns the XLink array position
    int getXLinkPos() const
    {
        return x_link_pos;
    }

/// Returns EXNumber     //added at v2.9.0 for clipboard storage
    int getEXNumber()
    {
        return ex_number;
    }

/// Returns CheckCount
    int getCheckCount() //added at v2.9.1
    {
        return check_count;
    }

/// Returns TrackVectorPosition
    unsigned int getTrackVectorPosition() const
    {
        return track_vector_pos;
    }

/// Returns signed integer value of TrackVectorPosition  (used in flip, mirror etc for pref dirs) added at v2.9.1
    int getSignedIntTrackVectorPosition() const
    {
        return track_vector_pos;
    }

/// Returns EXGraphicPtr for preferred directions
    // Graphics::TBitmap *GetEXGraphicPtr()
    // {
    //     return(getPrefDirGraphicPtr());
    // }

/// Returns route graphic
    // Graphics::TBitmap *GetRouteEXGraphicPtr()
    // {
    //     return(getRouteGraphicPtr(AutoSignals, PrefDirRoute));
    // }

/// Used in pasting pref dirs
    void setTrackVectorPosition(int tv_pos) // added at v2.9.0
    {
        track_vector_pos = tv_pos;
    }

    void setCheckCount(int check_count) //added at v2.9.1
    {
        check_count = check_count;
    }

/// Used in pasting pref dirs
    void setELink(int input) // added at v2.9.0
    {
        e_link = input;
    }

/// Used in pasting pref dirs
    void setELinkPos(int input) // added at v2.9.0
    {
        e_link_pos = input;
    }

/// Used in pasting pref dirs
    void setXLink(int input) // added at v2.9.0
    {
        x_link = input;
    }

/// Used in pasting pref dirs
    void setXLinkPos(int input) // added at v2.9.0
    {
        x_link_pos = input;
    }

/// Used in pasting pref dirs
    void setEXNumber(int input) // added at v2.9.0
    {
        ex_number = input;
    }

/// Used in pasting pref dirs
    // void setEXGraphicPtr(graphics::TBitmap *input) //added at v2.9.1
    // {
    //     EXGraphicPtr = input;
    // }

/// Used in pasting pref dirs
    // void setEntryDirectionGraphicPtr(graphics::TBitmap *input) //added at v2.9.1
    // {
    //     EntryDirectionGraphicPtr = input;
    // }

/// Default constructor, loads default values
//     PrefDirElement() : TrackElement(), ELink(-1), ELinkPos(-1), XLink(-1), XLinkPos(-1), EXNumber(-1), TrackVectorPosition(-1), CheckCount(0), EXGraphicPtr(0),
//         EntryDirectionGraphicPtr(0), IsARoute(false), AutoSignals(false), PrefDirRoute(false)
//     {
//         ;
//     }

// /// Constructs a PrefDirElement from a base TrackElement. Sets up the TrackElement values but leaves others as default values
//     PrefDirElement(tTrackElement input) : TTrackElement(Input), ELink(-1), ELinkPos(-1), XLink(-1), XLinkPos(-1), EXNumber(-1), TrackVectorPosition(-1),
//         CheckCount(0), EXGraphicPtr(0), EntryDirectionGraphicPtr(0), IsARoute(false), AutoSignals(false), PrefDirRoute(false)
//     {
//         ;
//     }

// external functions

    bool entryExitNumber();
///< determines and loads EXNumber (see above)
    std::string logPrefDir() const;
///< Sends a list of PrefDirElement values to Utilities->CallLog file for debugging purposes
    PrefDirElement(TrackElement input_element, int e_link, int e_link_pos, int x_link, int x_link_pos, int track_vector_pos);
///< Constructs a PrefDirElement from supplied values
//     Graphics::TBitmap *GetDirectionPrefDirGraphicPtr() const;
// ///< picks up the EntryDirectionGraphicPtr for preferred directions
//     Graphics::TBitmap *GetDirectionRouteGraphicPtr(bool autoSigsFlag, bool prefDirRoute) const;
// ///< picks up the green or red route direction graphic
//     Graphics::TBitmap *GetOriginalGraphicPtr();
// ///< picks up the original (non-flashing) graphic for use during route flashing
//     Graphics::TBitmap *GetPrefDirGraphicPtr();
// ///< picks up the EXGraphicPtr for preferred directions
//     Graphics::TBitmap *GetRouteAutoSigsGraphicPtr();
// ///< picks up the blue route graphic (not used - superseded by GetRouteGraphicPtr)
//     Graphics::TBitmap *GetRouteGraphicPtr(bool autoSigsFlag, bool prefDirRoute);
///< picks up the appropriate route graphic

};

// ---------------------------------------------------------------------------

/// The basic preferred direction class, consisting of any number of elements with preferred directions set. Used during setting up preferred directions and track lengths (constructPrefDir), and for all completed preferred directions in the railway (EveryPrefDir)
class OnePrefDir
{
public: // don't want descendant (tOneRoute) to access the PrefDir4MultiMap

    typedef std::multimap<HVPair, unsigned int, MapComp>PrefDir4MultiMap;
///< h_loc&v_loc as a pair, and PrefDirVectorPosition, can be up to 4 values at any H&V
    typedef std::multimap<HVPair, unsigned int, MapComp>::iterator PrefDir4MultiMapIterator;
    typedef std::pair<HVPair, unsigned int>PrefDir4MultiMapEntry;

    PrefDir4MultiMap pref_dir_4_mm;
///< the pref dir multimap - up to 4 values (up to 2 tracks per element each with 2 directions)

private:
// inline functions

/// Empty the existing vectors & map
    void clearPrefDir()
    {
        pref_dir_vec.clear();
        search_vec.clear();
        pref_dir_4_mm.clear();
    }

// functions defined in .cpp file

/// Retrieves a PrefDir4MultiMap iterator to the PrefDir element at PrefDirVectorPosition.  Used during ErasePrefDirElementAt to erase the relevant element in the multimap.  If nothing is found this is an error but the error message is given in the calling function.
    PrefDir4MultiMapIterator getExactMatchFrom4MultiMap(int caller, unsigned int prefdirvectorposition, bool &FoundFlag);
/// Store a single pref dir element in the vector & map
    void storePrefDirElement(int caller, PrefDirElement LoadPrefDirElement);
/// Erase a single element from PrefDirVector and 4MultiMap, decrementing the remaining PrefDirElementNumbers in 4MultiMap if they are greater than the erased value.
    void erasePrefDirElementAt(int caller, int prefdirvectorposition);
/// Diagnostic validity check
    void checkPrefDir4MultiMap(int caller);
/// Called after ErasePrefDirElementAt to decrement the remaining PrefDirElementNumbers in 4MultiMap if they are greater than the erased value.
    void decrementPrefDirElementNumbersInPrefDir4MultiMap(int caller, unsigned int erasedelementnumber);

protected: // descendant (tOneRoute) can access these

    static const int prefDirSearchLimit = 30000;
///< limit to the number of elements searched in attempting to find a preferred direction

// [dropped as not a good strategy because gaps interfered with direct line searches - instead introduced TotalSearchCount and now use that
// to limit searches. Leave in though in case rethink strategy later on]  Search limit values - set the H&V limits when searching for
// the next pref dir element (or route as inherited by TOneRoute), all points on search path must lie within 15 elements greater than
// the box of which the line between start and finish is a diagonal (else search takes too long)
    int searchLimitLowH;
    int searchLimitHighH;
    int searchLimitLowV;
    int searchLimitHighV;

    int totalSearchCount;
///< counts search elements, used to abort searches (prefdirs or routes) if reaches too high a value

// functions defined in .cpp file

/// Called by GetStartAndEndPrefDirElements...
/** which in turn is called by PresetAutoSigRoutesButtonClick. Checks for a diagonal link in
the autosigsroute being fouled by an adjacent track with a corresponding link that meets at the diagonal link, and if it is it
returns true and prevents the route being set.  Note that adjacent track consisting of buffers, gaps and continuations at the
diagonal link are also excluded though they need not be, but it makes the check code simpler and such adjacent track is untidy
and can be modelled better anyway.  Added at v2.1.0. */
    bool presetAutoRouteDiagonalFouledByTrack(int caller, PrefDirElement elementin, int xlink);
/// Checks ElementIn and returns true only if a single prefdir set at that H&V, with EntryPos giving entry position, not points, crossovers, signals with wrong direction set, or buffers. Added at v1.2.0
    bool presetAutoRouteElementValid(int caller, PrefDirElement elementin, int entrypos);
/// Try to find a selected element from a given start position.  Enter with CurrentTrackElement stored in the PrefDirVector, XLinkPos set to the link to search on, & SearchVector cleared unless entered recursively.  Function is a continuous loop that exits when find required element (returns true) or reaches a buffer or continuation or otherwise fails a search condition (returns false).
    bool searchForPrefDir(int caller, TrackElement track_element, int x_link_pos, int required_position);
/// Although there may be up to four entries at one H & V position this function gets just one. It is used in EraseFromPrefDirVectorAnd4MultiMap by being called as many times as there are PrefDir elements at H & V
    int getOnePrefDirPosition(int caller, int h_loc, int v_loc);
/// Called after a successful search to add the elements from the search vector to the pref dir vector
    void convertPrefDirSearchVector(int caller);

public:

//    typedef std::vector<TPrefDirElement> TPrefDirVector; //dropped here because used as a forward declaration earlier
    typedef std::vector<PrefDirElement>::iterator PrefDirVectorIterator;
    typedef std::vector<PrefDirElement>::const_iterator PrefDirVectorConstIterator;

    PrefDirVector pref_dir_vec, search_vec;
///< pref dir vectors, first is the main vector, second used to store search elements temporarily

// inline functions

/// Return the vector size
    unsigned int prefDirSize() const
    {
        return pref_dir_vec.size();
    }

/// Return the vector size
    unsigned int searchVectorSize() const
    {
        return search_vec.size();
    }

/// Empty the existing preferred direction vector & map - for use by other classes
    void externalClearPrefDirAnd4MultiMap()
    {
        clearPrefDir();
    }

    // functions defined in .cpp file

/// Determines whether the preferred direction pointed to has another pref dir in the opposite direction set (returns true) or not
    bool biDirectionalPrefDir(int caller, PrefDir4MultiMapIterator pd_ptr);
/// Called before PrefDir loading as part of the FileIntegrityCheck function in case there is an error in the file. Very similar to LoadPrefDir but with value checks instead of storage in PrefDirVector.
    bool checkOnePrefDir(int caller, int num_active_elements, std::ifstream &vec_file);
/// Used when setting preferred directions, true if able to finish at the last selected element (can't finish if there is only one element or if end on leading points)
    bool endPossible(int caller, bool &leading_points);
/// Finds a pref dir element that links to another element at given vector number and link number & position, returns true if found with linked vector number, true if buffer or continuation with link at blank end & linked vector number = -1, or false if not found with vector number == -1
    bool findLinkingPrefDir(int caller, int pref_dir_vector_num, int linknumberpos, int linknumber, int &linkedpref_dir_vector_num);
/// Finds a pref dir element that is compatible and links to another element at given vector number and link number & position, returns true if found with linked vector number, true if buffer or continuation with link at blank end & linked vector number = -1, or false if not found with vector number == -1
    bool findLinkingCompatiblePrefDir(int caller, int pref_dir_vector_num, int link_number_pos, int link_number, int &linkedpref_dir_vector_num);
/// Used when continuing a chain of preferred directions or element lengths. Tries to find a set of linked tracks between the last selected element and the one at h_loc & v_loc, and returns true if it finds one.  FinishElement is returned true if the element selected is a buffer or continuation - in which case the chain is complete
    bool getNextPrefDirElement(int caller, int h_loc, int v_loc, bool &FinishElement);
/// Called when searching for start and end PrefDirElements when setting up automatic signals routes in PreStart mode
    bool getStartAndEndPrefDirElements(int caller, PrefDirElement &startelement, PrefDirElement &endelement, int &lastiteratorvalue);
/// Used when beginning a chain of preferred directions or element lengths. Enter with h_loc & v_loc set to selected element & check if selected element is a valid track element, return false if not, if it is, store it as the first entry in PrefDirVector and return true
    bool getPrefDirStartElement(int caller, int h_loc, int v_loc);
/// Called during PrefDir build or distance setting. It truncates at & including the first element in the PrefDir vector that matches H & V. After the truncate the final element of the remaining PrefDir has its data members reset to the same defaults as would be the case if the PrefDir had been built up to that point - i.e. for first element or a leading point.
    bool getPrefDirTruncateElement(int caller, int h_loc, int v_loc);
/// Checks that all elements in PrefDirVector have been properly set, i.e. don't have their default values, and that every element is connected to the next element
    bool validatePrefDir(int caller);
/// Return the vector position of the last element in the vector (i.e. one less than the vector size)
    int lastElementNumber(int caller) const;
/// Return a pointer to the last element in the vector
    PrefDirVectorIterator lastElementPtr(int caller);
/// Return a non-modifiable element at PrefDirVector position 'At'
    const PrefDirElement &getFixedPrefDirElementAt(int caller, int at) const;
/// Return a modifiable element at PrefDirVector position 'At'
    PrefDirElement &getModifiablePrefDirElementAt(int caller, int at);
/// Return a non-modifiable element at SearchVector position 'At'
    const PrefDirElement &getFixedSearchElementAt(int caller, int at) const;
/// Return a modifiable element at SearchVector position 'At'
    PrefDirElement &getModifiableSearchElementAt(int caller, int at);
/// Used when setting element lengths, returns in &OverallDistance the overall distance for the selected chain of elements and also the speed limit in &OverallSpeedLimit, which is set to -1 if the speed limits vary over the chain
    void calcDistanceAndSpeed(int caller, int &overalldistance, int &overallspeedlimit, bool &LeadingPointsAtLastElement);

/// Store a single pref dir element in the vector & map - used by other classes
    void externalStorePrefDirElement(int caller, PrefDirElement load_pref_dir_element)
    {
        storePrefDirElement(6, load_pref_dir_element);
    }
/// Return up to 4 vector positions for a given h_loc & v_loc; unused values return -1
    void getVectorPositionsFromPrefDir4MultiMap(int caller, int h_loc, int v_loc, bool &foundflag, int &prefdirpos0, int &prefdirpos1, int &prefdirpos2,
                                                int &PrefDirPos3);
/// Old version of LoadPrefDir, used during development when the save format changed so the old files could be loaded prior to resaving in the new format
    void loadOldPrefDir(int caller, std::ifstream &VecFile);
/// Load a vector and map of preferred directions from the file
    void loadPrefDir(int caller, std::ifstream &VecFile);
/// PrefDir and route track marker, including direction markers.
/**  Function used for both PrefDirs (prefDirRoute == PrefDirCall) and routes (PrefDirRoute == RouteCall).
The graphics for marker colours and direction are already stored in all PrefDirElements in TOnePrefDir and TOneRoute, and this
function is called to display them, all in the case of a PrefDir, but for a route only the first and last elements have direction
markers. No markers are displayed if a train is present on an element.  Also no display if EXGraphicPtr not set.  If building a
PrefDir (buildingPrefDir true) then the start and end rectangles are also displayed. */
    // void prefDirMarker(int caller, PrefDirRoute prefdirroute, bool buildingprefdir, TDisplay *Disp) const;
/// Save the preferred direction vector to a file
    void savePrefDirVector(int caller, std::ofstream &vec_stream);
/// Save the search vector to a file
    void saveSearchVector(int caller, std::ofstream &vec_stream);
/// Used when creating a bitmap image to display preferred directions (as on screen during 'Set preferred direction' mode)
    // void writePrefDirToImage(int caller, Graphics::TBitmap *Bitmap);

// EveryPrefDir (declared in InterfaceUnit.h) functions (all external)

/// Check loaded PrefDir against loaded track, and if discrepancies found clear EveryPrefDir & PrefDir4MultiMap, messages are given by the calling routine.  Return true for OK
    bool checkPrefDirAgainstTrackVectorNoMessage(int caller);
/// Check loaded PrefDir against loaded track, and if discrepancies found give message & clear EveryPrefDir & PrefDir4MultiMap.
    void checkPrefDirAgainstTrackVector(int caller);
/// Used when a preferred direction has been set to add all the elements to EveryPrefDir, except when they already exist in EveryPrefDir
    void consolidatePrefDirs(int caller, std::shared_ptr<OnePrefDir> input_pref_dir);
/// Erase element at h_loc and v_loc from the PrefDirVector and from the 4MultiMap. Note that this entails erasing up to four elements (2 directions and 2 tracks for 4-entry elements).
    void eraseFromPrefDirVectorAnd4MultiMap(int caller, int h_loc, int v_loc);
/// Similar to PrefDirMarker but used only to mark EveryPrefDir - red for unidirectional PrefDir & green for bidirectional. Colours taken from the route colours. Plot red first so green overwrites for bidirectional points.
    // void everyPrefDirMarker(int caller, TDisplay *Disp);
/// After a track element is erased the preferred direction elements are likely to be affected. This function erases any preferred direction elements that either correspond to the erased track element, or were linked to it
    void realignAfterTrackErase(int caller, int erasedtrackvectorposition);
/// Called after the track vector has been rebuilt following linking, to rebuild the preferred direction vector to correspond to the element positions in the rebuilt track vector. Doesn't affect the preferred direction multimap.
    void rebuildPrefDirVector(int caller);
};

/// Allows a single Width x Height graphic to change and change back independently of the remaining display
/**
Used for the flashing green and red gap markers, flashing points and route start graphics.  The code is mostly self-explanatory, but SetScreenHVSource (sets source
rectangle) must be called before the original graphic is loaded, whether or not the graphic is loaded from the screen (using
LoadOriginalScreenGraphic, for point flashing and route start markers) or an existing bitmap (using loadOriginalExistingGraphic, for red
and green gap flashing), and OverlayGraphic and OriginalGraphic must be loaded before they are plotted.  Checks are
built in for these conditions.  SourceRect is the rectangle on the appropriate canvas where the original graphic is taken from.  The
original graphic can be taken from the screen - LoadOriginalScreenGraphic(), or from a section from an existing bitmap -
LoadOriginalExistingGraphic.  If an existing bitmap is selected then the loading function overrides the size that was set in the
constructor, and SourceRect & HPos & VPos that were set in SetScreenHVSource.
*/
class GraphicElement
{
private:

    bool overlay_plotted{false}, overlay_loaded{false}, original_loaded{false}, screen_source_set{false}, screen_graphic_loaded{false}, existing_graphic_loaded{false};
///< state flags
    int h_pos{0}, v_pos{0};
///< horizontal and vertical positions
    int width{0}, height{0};
///< dimensions in pixels
    // Graphics::TBitmap *OriginalGraphic, *OverlayGraphic;
///< original and temporary overlay graphics
    // TRect SourceRect;
///< source rectangle of the original graphic

public:
// inline functions
    int getHPos()
    {
        return h_pos;
    }

    int getVPos()
    {
        return v_pos;
    }

/// Set SourceRect member values from those supplied and existing Width & Height - ensure this is only called after Width & Height are set
    // void setSourceRect(int left, int top)
    // {
    //     SourceRect.init(left, Top, Left + Width, Top + Height);
    // }

// functions defined in .cpp file


    // void loadOriginalExistingGraphic(int caller, int h_offset, int v_offset, int width_in, int height_in, Graphics::TBitmap *Graphic);
///< Load red or green gap flashing graphic from the stored bitmaps
    void loadOriginalScreenGraphic(int caller);
///< Load original graphic from the screen for point flashing or route start markers
    // void loadOverlayGraphic(int caller, Graphics::TBitmap *Overlay);
///< Load the temporary overlay graphic
    // void plotOverlay(int caller, TDisplay *Disp);
///< Plot the overlay graphic on screen
    // void plotOriginal(int caller, TDisplay *Disp);
///< Plot the original graphic on screen
    void setScreenHVSource(int caller, int h_pos_in, int v_pos_in);
///< Set HPos, VPos & SourceRect member values from the supplied positions
    // TGraphicElement();
///< Default constructor (16 x 16 pixel element)
    // TGraphicElement(int widthin, int heightin);
///< Constructor for specified dimensions
//     ~TGraphicElement();
// ///< Destructor
};

// ---------------------------------------------------------------------------

/** Identification Number:
This was introduced when it was decided to have a route identification number for each route rather than using the vector
position number for identifying existing routes that were being extended during route building.  Using vector position numbers
meant that these identification numbers had to be changed when existing routes were erased by trains passing over them.  IDint is
used for StartSelectionRouteID and ReqPosRouteID (see tAllRoutes) and ensures that any confusion with the old vector position
numbers is picked up by the compiler. Note that the route's RouteID value is an 'int', not an 'IDInt', 'IDInt' is only used for
StartSelectionRouteID and ReqPosRouteID */
class IDInt
{
private:
    int internal_int;
///< the internal integer value represented by IDInt

public:
// all inline

/// get the internal integer
    int getInt() const
    {
        return internal_int;
    }

/// Equality comparator
    bool operator == (IDInt comparator)
    {
        return (internal_int == comparator.internal_int);
    }

/// Greater than comparator
    bool operator > (int comparator)
    {
        return (internal_int > comparator);
    }

/// Constructor that sets the internal integer to the input value. The 'explicit' prefix is used to force a compiler error if the input value is an IDInt, which would be a program error (otherwise it would be implicitly converted to an int)
    explicit IDInt(int integer)
    {
        internal_int = integer;
    }

/// Default constructor, internal integer set to -1
    IDInt()
    {
        internal_int = -1;
    }
};

// ---------------------------------------------------------------------------
// Track
// ---------------------------------------------------------------------------

/**All dynamic track data & methods.  Only one object since only one operating railway
Note:  The TrackMap & InactiveTrackMap were developed well after the TrackVector, to speed up track element
searches.  It was realised at that time that the maps themselves could contain type TTrackElement rather than int
(for trackVectorPosition), and that the track vectors could be dispensed with completely.  However after an attempt
to remove them it was clear that they were far too embedded throughout the program for easy removal, so they were
left in.  */
class Track
{
private:
/// Holds an array of TrackPieces, only accessible to TTrack
    class FixedTrackArray
    {
    public:

        FixedTrackPiece fixed_track_piece[FirstUnusedSpeedTagNumber];
///< the array member

/// Array constructor
        FixedTrackArray();
    };

    FixedTrackArray fixed_track_array;
///< the FixedTrackPiece array object
    TrackElement distance_start_element, distance_continuing_element;
///< initially used for track element lengths but since disused

    bool track_finished;
///< marker for all Conn & ConnLinkPos values set & track complete

    int gap_pos, gap_h_loc, gap_v_loc;
///< record gap setting info
    int h_loc_min{2000000000}, v_loc_min{2000000000}, h_loc_max{-2000000000}, v_loc_max{-2000000000};
///< give extent of railway for use in zoomed in and out displays and in saving railway images
    int link_check_array[9][2]{{1, 9}, {4, 6}, {7, 3}, {2, 8}, {0, 0}, {8, 2}, {3, 7}, {6, 4}, {9, 1}};
///< array of valid link connecting values, I don't think this is used now
    int link_hv_array[10][2]{{0, 0}, {-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {0, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}};

    /*
      Named Location Arrays:  Set out the adjacent positions and Types that are accepted as valid connections for
      a single location.  These are as follows:-
      Directly Adjacent = up, down, left or right - NOT diagonal.
      There are two separate groups, platforms, concourses & footcrossings (providing the crossing part touches or overlaps the other relevant
      named location) all link with each other providing directly adjacent, but not to NamedNonStationLocations.
      NamedNonStationLocation link to other NamedNonStationLocations providing directly adjacent, but not to anything else.
      //t     76
      //b     77
      //l     78
      //r     79
      //c     96
      //v fb  129
      //h fb  130
      //v underpass 145
      //h underpass 146
      //n     131
*/

///< array used to determine relative horizontal & vertical track element positions for specific link values
    int tag_76_array[25][3]{{-1, 0, 96}, // c    top plat
     {1, 0, 96}, {0, -1, 96}, {0, 1, 96}, {-1, 0, 76},    // t
     {1, 0, 76}, {0, -1, 76}, {0, 1, 76}, {-1, 0, 77},    // b
     {1, 0, 77}, {0, -1, 77}, {0, 1, 77}, {0, 0, 77}, {-1, 0, 78},    // l
     {1, 0, 78}, {0, -1, 78}, {0, 1, 78}, {-1, 0, 79},    // r
     {1, 0, 79}, {0, -1, 79}, {0, 1, 79}, {0, -1, 129},    // v fb
     {0, 0, 129}, {0, -1, 145},    // v up
     {0, 0, 145}};

///< these arrays give valid adjacent named element relative positions for each type of named element, the numbers - 76, 77 etc - relate to track element element SpeedTag values (76 - 79 = platforms, 96 = concourse, 129 & 130 = footbridges, 145 & 146 underpasses and 131 = non-station named location.
    int tag_77_array[25][3]{{-1, 0, 96}, // c     bot plat
     {1, 0, 96}, {0, -1, 96}, {0, 1, 96}, {-1, 0, 76},    // t
     {1, 0, 76}, {0, -1, 76}, {0, 1, 76}, {0, 0, 76}, {-1, 0, 77},    // b
     {1, 0, 77}, {0, -1, 77}, {0, 1, 77}, {-1, 0, 78},    // l
     {1, 0, 78}, {0, -1, 78}, {0, 1, 78}, {-1, 0, 79},    // r
     {1, 0, 79}, {0, -1, 79}, {0, 1, 79}, {0, 1, 129},    // v fb
     {0, 0, 129}, {0, 1, 145},    // v up
     {0, 0, 145}};

    int tag_78_array[25][3]{{-1, 0, 96}, // c     left plat
     {1, 0, 96}, {0, -1, 96}, {0, 1, 96}, {-1, 0, 76},    // t
     {1, 0, 76}, {0, -1, 76}, {0, 1, 76}, {-1, 0, 77},    // b
     {1, 0, 77}, {0, -1, 77}, {0, 1, 77}, {-1, 0, 78},    // l
     {1, 0, 78}, {0, -1, 78}, {0, 1, 78}, {-1, 0, 79},    // r
     {1, 0, 79}, {0, -1, 79}, {0, 1, 79}, {0, 0, 79}, {-1, 0, 130},    // h fb
     {0, 0, 130}, {-1, 0, 146},    // h up
     {0, 0, 146}};

    int tag_79_array[25][3]{{-1, 0, 96}, // c     right plat
     {1, 0, 96}, {0, -1, 96}, {0, 1, 96}, {-1, 0, 76},    // t
     {1, 0, 76}, {0, -1, 76}, {0, 1, 76}, {-1, 0, 77},    // b
     {1, 0, 77}, {0, -1, 77}, {0, 1, 77}, {-1, 0, 78},    // l
     {1, 0, 78}, {0, -1, 78}, {0, 1, 78}, {0, 0, 78}, {-1, 0, 79},    // r
     {1, 0, 79}, {0, -1, 79}, {0, 1, 79}, {1, 0, 130},    // h fb
     {0, 0, 130}, {1, 0, 146},    // h up
     {0, 0, 146}};

    int tag_96_array[28][3]{{-1, 0, 96}, // c       //concourse
     {1, 0, 96}, {0, -1, 96}, {0, 1, 96}, {-1, 0, 76},    // t
     {1, 0, 76}, {0, -1, 76}, {0, 1, 76}, {-1, 0, 77},    // b
     {1, 0, 77}, {0, -1, 77}, {0, 1, 77}, {-1, 0, 78},    // l
     {1, 0, 78}, {0, -1, 78}, {0, 1, 78}, {-1, 0, 79},    // r
     {1, 0, 79}, {0, -1, 79}, {0, 1, 79}, {0, 1, 129},    // v fb
     {0, -1, 129}, {1, 0, 130},    // h fb
     {-1, 0, 130}, {0, 1, 145},    // v up
     {0, -1, 145}, {1, 0, 146},    // h up
     {-1, 0, 146}};

    int tag_129_array[8][3] =// vert fb
    {{0, -1, 96}, // c
     {0, -1, 77}, // b
     {0, -1, 129},// v fb

     {0, 1, 96},  // c
     {0, 1, 76},  // t
     {0, 1, 129}, // v fb

     {0, 0, 76},  // t
     {0, 0, 77}}; // b

    int tag_145_array[8][3] =// vert up
    {{0, -1, 96}, // c
     {0, -1, 77},    // b
     {0, -1, 145},    // v fb

     {0, 1, 96},    // c
     {0, 1, 76},    // t
     {0, 1, 145},    // v fb

     {0, 0, 76},    // t
     {0, 0, 77}};    // b

    int tag_130_array[8][3] = // hor fb
    {{-1, 0, 96}, // c
     {-1, 0, 79},    // r
     {-1, 0, 130},    // h fb

     {1, 0, 96},    // c
     {1, 0, 78},    // l
     {1, 0, 130},    // h fb

     {0, 0, 78},    // l
     {0, 0, 79}};    // r

    int tag_131_array[4][3] = {{-1, 0, 131}, // n
     {1, 0, 131}, {0, -1, 131}, {0, 1, 131}};

    int tag_146_array[8][3] = // hor up
    {{-1, 0, 96}, // c
     {-1, 0, 79},    // r
     {-1, 0, 146},    // h fb

     {1, 0, 96},    // c
     {1, 0, 78},    // l
     {1, 0, 146},    // h fb

     {0, 0, 78},    // l
     {0, 0, 79}};    // r

    std::set<int> top_plat_allowed, bot_plat_allowed, left_plat_allowed, right_plat_allowed, name_allowed, level_crossing_allowed;
///< sets of valid TrackElements for placement of platforms and non-station named locations

public:
/*
All LCs begin with barriers raised. i.e. closed to trains, that is the normal state.  When a route is set through an LC an active LC object is created
by SetLCChangeValues (called by ConvertandAdd....  for lowering barriers) and added to the
ChangingLCVector.  Once created 'FlashingGraphics' takes care of the flashing, until the duration is reached.  While flashing no further routes
can be set through that LC and the first route can't be cancelled, hence the flashing only needs to cater for plotting the route on the one track that
started the barrier lowering.  When the duration is reached, the object is transferred to a new vector BarriersDownVector, after the StartTime has been
reset (to time the period for which the barriers are down - penalties are given for > 3 minutes), BarrierState changed to Down, and the object erased
from ChangingLCVector.  When there is no route through an LC and no train on the track then the barriers are raised - in ClockTimer2 - when the
BarriersDownVector object is copied back to ChangingLCVector with a new StartTime, BarrierState and ChangeDuration.  Again FlashingGraphics takes care
of the flashing until the duration is reached, when the object is erased from the vector and the LC reverts to its normal (barriers raised) state.
At v2.6.0 LCs could be lowered and raised manually, manual LCs are shown lowering and down as green and indicated by TypeOfRoute being 2.  A manual LC
can't have a route set while changing; can't be opened while a route is set; and must be opened manually.*/

    enum class BarrierState
///< state of barriers, values for level crossings either changing state or with barriers up or down
    {
        Raising, Lowering, Up, Down
    };

    class ActiveLevelCrossing
    {
    public:

        int type_of_route{0};
///< route type - 0 = nonsignals, 1 = preferred direction (can't have autosigs), 2 no route, 2 added at v2.6.0 for manual operation
        bool reduced_time_penalty{false};
///< marker that is set when a train is present on one of the elements of the LC - used to provide a 3 minute penalty allowance
        BarrierState barrier_state{BarrierState::Up};
///< state of barriers - Raising, Lowering, Up, Down (an enum - see above)
        float change_duration{0.};
///< duration of the level crossing changing period
        int base_element_speed_tag{1};
///< SpeedTag value for the base element of a level crossing
        int h_loc{0};
///< h_loc value for found level crossing element
        int v_loc{0};
///< v_loc value for found level crossing element
        boost::posix_time::time_duration start_time{0, 0, 0, 0};
///< stores the starting time for level crossing changing
    };

    typedef std::vector<ActiveLevelCrossing>ActiveLCVector;
///< vector of changing level crossing objects.  Note that although a LC may contain several elements there will be only one in the vector when changing, and it might be any of the individual elements.  This is because when an entry is made all linked elements have their attributes changed to 2 for changing, so no more are found.  This applies both for closing & opening to trains

    typedef std::vector<int>LCVector;
///< vector of level crossing InactiveTrackVector positions - note that this contains all LC elements whether linked to others or not

    // typedef std::vector<UserGraphicItem>UserGraphicVector;
///< vector of UserGraphicItems

    typedef std::vector<TrackElement>TrackVector;
///< vector of TrackElements
    typedef std::vector<TrackElement>::iterator TrackVectorIterator;
///< iterator for TTrackVector

    // typedef std::map<std::string, TPicture*>TUserGraphicMap;
///< map of filenames as key and TPicture* as value. This holds all the TPicture pointers created when a user graphic is selected
    // typedef std::pair<std::string, TPicture*>TUserGraphicMapEntry;
///<an entry for TUserGraphicMap

    typedef std::map<HVPair, unsigned int, MapComp>TrackMap;
///< map of TrackElement TrackVectorPositions, h_loc & v_loc pair is the key
    typedef TrackMap::iterator TrackMapIterator;
    typedef std::pair<HVPair, unsigned int>TrackMapEntry;

    typedef std::map<HVPair, HVPair, MapComp>GapMap;
///< map of matching gap positions as an h_loc/v_loc pair, with the key being
    typedef GapMap::iterator GapMapIterator;
///< the first gap h_loc/v_loc pair, contains one entry for each pair of matched gaps
    typedef std::pair<HVPair, HVPair>TGapMapEntry;

    typedef std::multimap<HVPair, unsigned int, MapComp>InactiveTrack2MultiMap;
///< multimap of inactive TrackElements (platforms, concourses, non-station named locations, parapets & level crossings) '2' because can have 2 entries (platforms) at a single location
    typedef InactiveTrack2MultiMap::iterator InactiveTrack2MultiMapIterator;
///< iterator for TInactiveTrack2MultiMap
    typedef std::pair<InactiveTrack2MultiMapIterator, InactiveTrack2MultiMapIterator>InactiveTrackRange;
///< range for TInactiveTrack2MultiMap

    typedef std::pair<unsigned int, unsigned int>IMPair;
///< TrackElement pair type used for inactive elements, values are vector positions

    typedef std::list<int>LNPendingList;
///< type list of location name vector positions (see note below) used during
    typedef LNPendingList::iterator TLNPendingListIterator;
///< naming of linked named location elements

    typedef std::multimap<HVPair, int, MapComp>LNDone2MultiMap;
///< multimap of location name vector positions (see note below) used
    typedef LNDone2MultiMap::iterator LNDone2MultiMapIterator;
///< during naming of linked named location elements, '2' because there
    typedef std::pair<HVPair, int>LNDone2MultiMapEntry;
///< can be up to 2 entries (platforms) at a single location

    typedef std::multimap<std::string, int>LocationNameMultiMap;
///< map of location name vector positions (see note below), one entry for every element that is a FixedNamedLocationElement i.e platforms, concourses, footcrossings & named non-station locations.  Hence the only active track elements included are footcrossings
    typedef LocationNameMultiMap::iterator LocationNameMultiMapIterator;
    typedef std::pair<LocationNameMultiMapIterator, LocationNameMultiMapIterator> LocationNameMultiMapRange;
    typedef std::pair<std::string, int>LocationNameMultiMapEntry;

    typedef std::map<HVPair, bool> HVPairsLinkedMap;
///< added at v2.6.1 for use in PopulateHVPairsLinkedMapAndNoDuplicates

// NOTE: the above (tLNPendingList, TLNDone2MultiMap & TLocationNameMultiMap) store adjusted vector positions - adjusted because have
// a single int to represent both active and inactive vector positions.  Use (-1 - Position) for active vector positions & (Position)
// for inactive vector positions (most location elements are in the inactive vector so these are positive).  The '-1' bit is needed
// because the value '0' is used for the first position in the inactive vector

    typedef std::map<std::string, int>ActiveTrackElementNameMap;
///< map of ActiveTrackElementNames compiled and used for populating the LocationNameComboBox during timetable creation or editing.  Used in place of LocationNameMultiMap as that can contain concourses and non-station named locations that aren't associated with any track.  The second 'int' entry is a dummy, only the list of std::string names is needed, and being a map it is automatically sorted and without duplicates.
    typedef ActiveTrackElementNameMap::iterator ActiveTrackElementNameIterator;
    typedef std::pair<std::string, int>ActiveTrackElementNameMapEntry;
    // typedef std::map<HVPair, Graphics::TBitmap*> MultiplayerOverlayMap; //added for multiplayer

    struct InfrastructureFailureEntry //added at v2.13.0
    {
        int tv_pos;
        boost::posix_time::time_duration failure_time;
        boost::posix_time::time_duration repair_time;
    };

    typedef std::vector<InfrastructureFailureEntry>FailedElementVector; //added at v2.13.0

    typedef std::vector<int> SimpleVector; //added at v2.13.0

/// Used as basic elements in a table of signals - see SigTable below
    struct SigElement
    {
        // NOTE: Don't alter the order of these members as they are loaded from an array of values in the constructor
        int speedTag;
///< the TrackElement SpeedTag value - specifies the signal element
        int attribute;
///< the signal state - red, yellow, double yellow or green
        // Graphics::TBitmap* SigPtr;
///< pointer to the graphic
    };

    SigElement sig_table[40];
///< original table of signals for four aspect
    SigElement sig_table_three_aspect[40];
///< new at version 0.6 for three aspect
    SigElement sig_table_two_aspect[40];
///< new at version 0.6 for two aspect
    SigElement sig_table_ground_signal[40];
///< new at version 0.6 for ground signals

    SigElement failed_sig_table[8];
///< table of failed signals added at v2.13.0

    std::string route_fail_message;

    std::string nl{"\n"};

    bool active_track_element_name_compile{false};
///< indicates that the ActiveTrackElementNameMap has been compiled
    bool copy_flag{false};
///< true only when copying a selection, used to prevent location names being copied
    bool duplicatedLocationName(int caller, bool give_message);
///< examines LocationNameMultiMap and returns true if there are two or more locations with the same name - added at v2.6.1 to cater for Bill78's new .dev file merge program and used when try to save as a .rly file.
    bool gap_flash_flag{false};
///< true when a pair of connected gaps is flashing
    bool lc_change_flag{false};
///< true when LCs changing
    bool lc_found_in_auto_sigs_route{false};
///< true if found an LC during an automatic route search
    bool no_plats_message_sent{false};
///< used to send no platforms warning once only
    bool suppress_routes_fail_msg{false};
///< true if a message has been given in the search routine, to avoid giving multiple times and to avoid other failure messages being given
    bool lc_found_in_route_building_flag{false};
///< true if a route set through an LC that is closed to trains (& therefore needs to be opened)
    bool point_flash_flag{false};
///< true when points are flashing during manual change
    bool route_flash_flag{false};
///< true while a route is flashing prior to being set
    bool skip_location_name_mm_check{false};
///<changed from PastingWithAttributes in v2.4.0 as all pastes are now with attributes - needed to suppress multimap checks while pasting
    bool override_and_hide_signal_bridge_msg{false};
///<if false signals facing bridges are not permitted, but can be set to true using CTRL ALT 5
    bool signal_failed_flag{false};
///<indicates at least one signal has failed
    bool tsr_flag{false};
///<indicates at least one element has a temporary speed restriction
    float level_crossing_barrier_up_flash_duration{false};
///< duration of the flash period when level crossing closing to trains
    float level_crossing_barrier_down_flash_duration{false};
///< duration of the flash period when level crossing opening
    int flip_array[FirstUnusedSpeedTagNumber]{
        0, 1, 2, 5, 6, 3, 4, 9, 10, 7, 8, 13, 14, 11, 12, 15, 16, 17, 19, 18, 22, 23, 20, 21, 26, 27, 24, 25, 30, 31, 28, 29, 34, 35, 32, 33, 38, 39, 36, 37, 42,
        43, 40, 41, 45, 44, 47, 46, 48, 49, 51, 50, 53, 52, 55, 54, 57, 56, 59, 58, 60, 61, 63, 62, 66, 67, 64, 65, 68, 69, 71, 70, 74, 75, 72, 73, 77, 76, 78,
        79, 80, 81, 83, 82, 86, 87, 84, 85, 88, 89, 91, 90, 94, 95, 92, 93, 96, 99, 100, 97, 98, 103, 104, 101, 102, 106, 105, 109, 110, 107, 108, 113, 114,
        111, 112, 117, 118, 115, 116, 119, 120, 121, 123, 122, 124, 125, 126, 128, 127, 129, 130, 131, 134, 133, 132, 135, 139, 138, 137, 136, 143, 142, 141,
        140, 144, 145, 146
    };
///< holds TrackElement SpeedTag values for 'flipping' via menu items 'Edit' & 'Flip'
    int gap_flash_green_pos{0}, gap_flash_red_pos{0};
///< TrackVectorPosition of the gap element that is flashing green or red
    int mirror_array[FirstUnusedSpeedTagNumber]{
        0, 1, 2, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 15, 16, 17, 19, 18, 21, 20, 23, 22, 25, 24, 27, 26, 29, 28, 31, 30, 33, 32, 35, 34, 37, 36, 39, 38, 41,
        40, 43, 42, 45, 44, 47, 46, 48, 49, 51, 50, 53, 52, 55, 54, 57, 56, 59, 58, 61, 60, 62, 63, 65, 64, 67, 66, 69, 68, 70, 71, 73, 72, 75, 74, 76, 77, 79,
        78, 81, 80, 82, 83, 85, 84, 87, 86, 89, 88, 90, 91, 93, 92, 95, 94, 96, 98, 97, 100, 99, 102, 101, 104, 103, 106, 105, 108, 107, 110, 109, 112, 111,
        114, 113, 116, 115, 118, 117, 119, 120, 124, 122, 123, 121, 126, 125, 127, 128, 129, 130, 131, 132, 135, 134, 133, 137, 136, 139, 138, 142, 143, 140,
        141, 144, 145, 146
    };
///< holds TrackElement SpeedTag values for 'mirroring' via menu items 'Edit' & 'Mirror'
    int rot_right_array[FirstUnusedSpeedTagNumber]{
        0, 2, 1, 4, 6, 3, 5, 14, 12, 13, 11, 7, 9, 8, 10, 15, 16, 17, 19, 18, 25, 27, 24, 26, 21, 23, 20, 22, 35, 33, 34, 32, 28, 30, 29, 31, 41, 43, 40, 42, 37,
        39, 36, 38, 46, 47, 44, 45, 49, 48, 51, 50, 56, 57, 58, 59, 52, 53, 54, 55, 63, 62, 60, 61, 65, 67, 64, 66, 71, 70, 68, 69, 73, 75, 72, 74, 79, 78, 76,
        77, 83, 82, 80, 81, 85, 87, 84, 86, 91, 90, 88, 89, 93, 95, 92, 94, 96, 102, 104, 101, 103, 98, 100, 97, 99, 106, 105, 108, 110, 107, 109, 116, 118,
        115, 117, 112, 114, 111, 113, 120, 119, 122, 124, 121, 123, 127, 128, 126, 125, 130, 129, 131, 133, 134, 135, 132, 137, 138, 139, 136, 143, 142, 140,
        141, 144, 146, 145
    };
///< holds TrackElement SpeedTag values for 'rotating right' via menu items 'Edit' & 'Rotate right'
    int rot_left_array[FirstUnusedSpeedTagNumber]{
        0, 2, 1, 5, 3, 6, 4, 11, 13, 12, 14, 10, 8, 9, 7, 15, 16, 17, 19, 18, 26, 24, 27, 25, 22, 20, 23, 21, 32, 34, 33, 35, 31, 29, 30, 28, 42, 40, 43, 41, 38,
        36, 39, 37, 46, 47, 44, 45, 49, 48, 51, 50, 56, 57, 58, 59, 52, 53, 54, 55, 62, 63, 61, 60, 66, 64, 67, 65, 70, 71, 69, 68, 74, 72, 75, 73, 78, 79, 77,
        76, 82, 83, 81, 80, 86, 84, 87, 85, 90, 91, 89, 88, 94, 92, 95, 93, 96, 103, 101, 104, 102, 99, 97, 100, 98, 106, 105, 109, 107, 110, 108, 117, 115,
        118, 116, 113, 111, 114, 112, 120, 119, 123, 121, 124, 122, 128, 127, 125, 126, 130, 129, 131, 135, 132, 133, 134, 139, 136, 137, 138, 142, 143, 141,
        140, 144, 146, 145
    };
///< holds TrackElement SpeedTag values for 'rotating left' via menu items 'Edit' & 'Rotate left'
    FailedElementVector failed_points, failed_signals, tsr;
///< vector of failed points with track vector positions & repair times for use in failure handling (new at v2.13.0)
    std::map<std::string, char>continuation_name;
///< map of all continuation names, char is a dummy
    //MultiplayerOverlayMap MultiplayerOverlayMap; //added for multiplayer
///< map of coupled continuations
    ActiveTrackElementNameMap active_track_element_name;
///< map of active track element names
    ActiveLCVector changing_lc;
///< vector of values for changing level crossings - i.e. barriers in course of being raised or lowered
    ActiveLCVector barriers_down;
///< vector of LCs with barriers down
    GapMap gap_map;
///< map of gaps (see type for more information above)
    //TGraphicElement *GapFlashGreen, *GapFlashRed;
///< the red & green circle graphics used to show where the gaps are
    InactiveTrack2MultiMap inactive_track2_mm;
///< multimap of inactive TrackElements (see type for more information above)
    LCVector lc;
///< vector of level crossing InactiveTrackVector positions
    LNDone2MultiMap ln_done_2_mm;
///< multimap of processed location name elements (see type for more information above)
    LNPendingList ln_pending_list;
///< list of location name elements awaiting processing (see type for more information above)
    LocationNameMultiMap location_name_mm;
///< multimap of location names (see type for more information above)
    SimpleVector simple_vec;
///< vector of simple element track vector positions
    // UserGraphicVector user_graphics, select_graphics;
///< vectors of user graphics
    //UserGraphicMap user_graphic_map;
///<the map of graphic filenames as key and TPicture* as values
    TrackMap track_map;
///< map of track (see type for more information above)
    TrackVector tracks, inactive_tracks, new_vec, distance_vec, distance_search, select;
///< vectors of TrackElements
    TrackVectorIterator next_element;
///< track vector iterator used during cycling through a track vector
    //UserGraphicMapEntry ugme;
///<an entry for the UserGraphicMap

// inline functions

/// Return location name for a given inactive track vector position
    std::string getLocationName(unsigned int inactive_track_element_pos)
    {
        return inactiveTrackElementAt(24, inactive_track_element_pos).location_name;
    }

/// Indicates whether or not the railway is ready for saving as a '.rly' file and for operation
    bool isReadyForOperation(bool give_message)
    {
        return (isTrackFinished() && !locationsNotNamed(1) && !gapsUnset(8) && !duplicatedLocationName(0, give_message));
    }

/// Indicates whether or not the track has been successfully linked together
    bool isTrackFinished()
    {
        return track_finished;
    }

/// checks if a user graphic present
    // bool userGraphicPresentAtHV(int caller, int h_pos, int v_pos, int& ugi_vector_pos)
    // {
    //     UGIVectorPos = 0;
    //     for(int x = (UserGraphicVector.size() - 1); x >= 0; x--) // go downwards because may erase the element identified
    //     {
    //         if((h_pos >= (UserGraphicVectorAt(18, x).HPos - (Display->DisplayOffsetH * 16))) && (HPos < (UserGraphicVectorAt(19, x).HPos +
    //             UserGraphicVectorAt(20, x).Width - (Display->DisplayOffsetH * 16))) && (VPos >= (UserGraphicVectorAt(21, x).VPos -
    //             (display->DisplayOffsetV * 16))) && (VPos < (UserGraphicVectorAt(22, x).VPos + UserGraphicVectorAt(23, x).Height -
    //             (display->DisplayOffsetV * 16))))
    //         {
    //             UGIVectorPos = x;
    //             return(true);
    //         }
    //     }
    //     return(false);
    // }

    enum
    {
        FourAspectBuild, ThreeAspectBuild, TwoAspectBuild, GroundSignalBuild
    } signal_aspect_build_mode;
///< aspect mode for future signal additions

    int getGaph_loc()
    {
        return gap_h_loc;
    } // return the respective values

    int getGapv_loc()
    {
        return gap_v_loc;
    }

    int geth_locMax()
    {
        return h_loc_max;
    }

    int geth_locMin()
    {
        return h_loc_min;
    }

    int getv_locMax()
    {
        return v_loc_max;
    }

    int getv_locMin()
    {
        return v_loc_min;
    }

/// Return the corresponding link position (track always occupies either links 0 & 1 or 2 & 3)
    int getNonPointsOppositeLinkPos(int link_pos_in)
    {
        switch (link_pos_in) {
            case 3:
                return 2;
                break;
            case 2:
                return 3;
                break;
            case 1:
                return 0;
                break;
            default:
                return 1;
        }
    }

/// Return the number of active track elements
    int trackVectorSize()
    {
        return tracks.size();
    }

/// Return a basic track element from the SpeedTag   new at v2.2.0 - needed because Interface doesn't have direct access to FixedTrackArray
    TrackElement buildBasicElementFromSpeedTag(int caller, int speed_tag)
    {
        return fixed_track_array.fixed_track_piece[speed_tag];
    }

/// Return the number of selected active and inactive track elements (via menu items 'Edit' and 'Select')
    unsigned int selectVectorSize()
    {
        return select.size();
    }

/// Store a TrackElement in the SelectVector
    void selectPush(TrackElement track_element)
    {
        select.push_back(track_element);
    }

    void selectVectorClear()
    {
        select.clear();
    }

// set member values
    void seth_locMax(int h_loc)
    {
        h_loc_max = h_loc;
    }

    void seth_locMin(int h_loc)
    {
        h_loc_min = h_loc;
    }

    void setTrackFinished(bool value)
    {
        track_finished = value;
    }

    void setv_locMax(int v_loc)
    {
        v_loc_max = v_loc;
    }

    void setv_locMin(int v_loc)
    {
        v_loc_min = v_loc;
    }

// externally defined functions

/// Used to check the validity of footcrossing links
    bool activeMapCheck(int caller, int h_loc, int v_loc, int speed_tag);
/// Checks BarrierDownVector and returns true if there is one that is linked to the LC at H & V positions and is set to manual (typeOfRoute == 2), and returns the vector position in BDVectorPos
    bool anyLinkedBarrierDownVectorManual(int caller, int h_loc, int v_loc, int &bd_vector_pos);
/// True if a route or train present on any linked level crossing element
    bool anyLinkedLevelCrossingElementsWithRoutesOrTrains(int caller, int h_loc, int v_loc, PrefDirVector search_vec, bool &train_present);
/// Used during location naming to check for adjacent named elements to a given element at h_loc & v_loc with a specific SpeedTag, and if found allow that element to be inserted into the LNPendingList for naming similarly
    bool adjElement(int caller, int h_loc, int v_loc, int speed_tag, int &found_element);
/// Used in SearchForAndUpdateLocationName to check for adjacent named elements to a given element at h_loc & v_loc with a specific SpeedTag, and if found allow that name to be used for this element and all other named elements that are linked to it
    bool adjNamedElement(int caller, int h_loc, int v_loc, int speed_tag, std::string &location_name, int &found_element);
/// True for a blank (speedTag == 0) element at a specific Trackvector position, no longer used after TrackErase changed (now eraseTrackElement) so that blank elements aren't used
    bool blankElementAt(int caller, int at_pos) const;
// True if BarriersDownVector checks OK in SessionFile
    bool checkActiveLCVector(int caller, std::ifstream &vec_stream);
/// True if a footcrossing is linked properly at both ends
    bool checkFootCrossingLinks(int caller, TrackElement &track_element);
/// True if TrackElements in the file are all valid
    bool checkTrackElementsInFile(int caller, int &number_of_active_elements, bool &graphics_follow, std::ifstream& vec_stream);
///checks all user graphics & returns true for success
    bool checkUserGraphics(int caller, std::ifstream &in_stream, std::filesystem::path &graphics_path);
/// As DiagonalFouledByRouteOrTrain (in tAllRoutes) but only checks for a train (may or may not be a route present (new at v1.2.0))
    bool diagonalFouledByTrain(int caller, int h_loc, int v_loc, int diagonal_link_number, int &train_id);
/// True if the element defined by MapPos is present in LNDone2MultiMap, used during location naming
    bool elementInLNDone2MultiMap(int caller, int map_pos);
// True if the element defined by MapPos is present in LNPendingList, used during location naming
    bool elementInLNPendingList(int caller, int map_pos);
/// Check for track errors prior to gap setting - disused as incorporated a time-consuming double brute force search
    bool errorInTrackBeforeSetGaps(int caller, int &h_loc, int &v_loc);
/// True if there is an unset gap, and if so it is marked with a red circle, used during gap setting
    bool findAndHighlightAnUnsetGap(int caller);
/// Used in locating the screen name position for a named location, return true if find an inactive element called 'Name'
    bool findHighestLowestAndLeftmostNamedElements(int caller, std::string &name, int &v_pos_hi, int &v_pos_lo, int &h_pos);
/// True if find a non-platform element at h_loc & v_loc, and if so return its TrackVector position and a reference to it in TrackElement
    bool findNonPlatformMatch(int caller, int h_loc, int v_loc, int &position, TrackElement &track_element);
/// True if find an unset gap that matches the gap at h_loc & v_loc, if find one mark it with a green circle
    bool findSetAndDisplayMatchingGap(int caller, int h_loc, int v_loc);
/// True if there are gaps in the railway and any are unset
    bool gapsUnset(int caller);
/// Used to check the validity of footcrossing links
    bool inactiveMapCheck(int caller, int h_loc, int v_loc, int speed_tag);
/// New at v1.2.0; true if an inactive track element present
    bool inactiveTrackElementPresentAtHV(int caller, int h_loc, int v_loc);
/// True if there is an element adjacent to LinkIn for element at h_loc & v_loc
    bool isATrackElementAdjacentToLink(int caller, int h_loc_in, int v_loc_in, int link_in);
/// True if there is a vector entry at H & V that is set to manual (typeOfRoute == 2) and returns the vector position in BDVectorPos
    bool isBarrierDownVectorAtHVManual(int caller, int h_loc, int v_loc, int &bc_vector_pos);
/// True if track at link positions [0] & [1] if FirstTrack true, else that at [2] & [3] in TrackElement has the default length
/// and speed limit, return true if so
    bool isElementDefaultLength(int caller, TrackElement &track_element, bool first_track, bool &length_different, bool &speed_different);
/// True if a non-station named location at h_loc & v_loc
    bool isNamedNonStationLocationPresent(int caller, int h_loc, int v_loc);
/// True if a level crossing is found at H & V
    bool isLCAtHV(int caller, int h_loc, int v_loc);
/// True if an open (to trains) level crossing is found at H & V
    bool isLCBarrierDownAtHV(int caller, int h_loc, int v_loc);
/// True if a closed (to trains) level crossing is found at H & V
    bool isLCBarrierUpAtHV(int caller, int h_loc, int v_loc);
/// True if barrier is in process of opening or closing at H & V
    bool isLCBarrierFlashingAtHV(int caller, int h_loc, int v_loc);
/// True if a non-station named location or platform at h_loc & v_loc
    bool isPlatformOrNamedNonStationLocationPresent(int caller, int h_loc, int v_loc);
/// True if track has been successfully linked (not used any more)
    bool isTrackLinked(int caller);
/// checks for a route being set across an LC to prevent barriers raising
    bool lCInSearchVector(int caller, int h_loc, int v_loc, PrefDirVector SearchVector);  //added at v2.8.0
/// Attempt to link the track and return true if successful, if unsuccessful return error flag and position of the first element that can't be linked together with an appropriate message. This is a link checking (finalCall false) or linking (FinalCall true) function, with messages, called by TryToConnectTrack, which handles linking and all other associated functions
    bool linkTrack(int caller, bool &locerror, int &h_loc, int &v_loc, bool finalCall);
/// Attempt to link the track and return true if successful, don't issue any screen messages. This is also a link checking (finalCall false) or linking (FinalCall true) function, without messages,  that is called by TryToConnectTrack, which handles linking and all other associated functions
    bool linkTrackNoMessages(int caller, bool finalCall);
/// True if a non-empty LocationName found in LocationNameMultiMap
    bool locationNameAllocated(int caller, std::string LocationName);
/// True if there are unnamed NamedLocationElements (includes footcrossings)
    bool locationsNotNamed(int caller);
/// True if the two vector positions are points that have a straight and a diverging leg, are linked together by their
/// diverging legs, and both are set either to straight or to diverge
    bool matchingPoint(int caller, unsigned int trackvectorposition, unsigned int divergingposition);
/// True if the active or inactive TrackElement at h_loc & v_loc has its FixedNamedLocationElement member true
    bool namedLocationElementAt(int caller, int h_loc, int v_loc);
/// True if there is no active or inactive track in the railway
    bool noActiveOrInactiveTrack(int caller);
/// True if there is no active track in the railway
    bool noActiveTrack(int caller);
/// True if there are no gaps
    bool noGaps(int caller);
/// True if there are no NamedLocationElements (includes footcrossings)
    bool noNamedLocationElements(int caller);
/// True if there is a platform, NamedNonStationLocation or Concourse present in the railway
    bool nonFootCrossingNamedLocationExists(int caller);
/// True if there is at least one named location element with name 'LocationName', used in timetable integrity checking
    bool oneNamedLocationElementAtLocation(int caller, std::string LocationName);
/** check sufficient track elements with same LocationName linked together without any trailing point links to allow a train split. Only one
length is needed to return true, but this doesn't mean that all platforms at the location are long enough.  When a split is required during
operation a specific check is made using ThisNamedLocationLongEnoughForSplit.  Need at least two linked track elements with the same LocationName, with connected
elements at each end, which may or may not be named and no connections via point trailing links.  Note that these conditions
exclude opposed buffers since these not linked.  Used in timetable integrity checking. */
    bool oneNamedLocationLongEnoughForSplit(int caller, std::string LocationName);
/// True if another train on NextEntryPos track of element at NextPos, whether bridge or not, return false if not, or if NextPos == -1, or if only own train on the track
    bool otherTrainOnTrack(int caller, int nextpos, int nextentrypos, int owntrainid);
/// Check whether there is a platform present at h_loc & v_loc at the same side as the signal represented by SpeedTag, if so return true, and also return a pointer to the appropriate platform graphic (same as a normal platform graphic but with a bit cut out for the signal)
    //bool platformOnSignalSide(int caller, int h_loc, int v_loc, int speedtag, Graphics::TBitmap* &SignalPlatformGraphic);
/// Used in checking for duplicate location names after Bill78 (discord name) developed the .dev file merge tool - added at v2.6.1
    bool populateHVPairsLinkedMapAndNoDuplicates(int caller, LocationNameMultiMapRange ln_mmrg);
/// When track is being built it is entered into the TrackVector in the order in which it is built, and the TrackMap reflects that positioning.  When the track is linked, the vector is rebuilt in track element position order, and the map is also rebuilt to reflect the new positions.  Called during track linking, returns true if successful.
    bool repositionAndMapTrack(int caller);
/// Sets all Conns and CLks to -1 except for gapjumps that match and are properly set, returns true for any unset gaps
    bool resetConnClkCheckUnsetGapJumps(int caller);
/// Called by RepositionAndMapTrack to reset the connecting elements of all set gaps (their trackVector positions will have changed during the repositioning process), returns true if successful
    bool resetGapsFromGapMap(int caller);
/// Return a reference to the inactive track element pointed to by NextTrackElementPtr (during zoomed-in or out track rebuilding, or writing image files), return true if there is a next one or false if not
    bool returnNextInactiveTrackElement(int caller, TrackElement &next);
/// Return a reference to the active track element pointed to by NextTrackElementPtr (during zoomed-in or out track rebuilding, or writing image files), return true if there is a next one or false if not
    bool returnNextTrackElement(int caller, TrackElement &next);
/// See above under 'OneNamedLocationLongEnoughForSplit'
    bool thisNamedLocationLongEnoughForSplit(int caller, std::string locationname, int firstnamedelementpos, int &secondnamedelementpos,
                                             int &FirstNamedLinkedElementPos, int &SecondNamedLinkedElementPos);
/// True if a non-empty LocationName found as a timetabled location name i.e. not as a continuation name
    bool timetabledLocationNameAllocated(int caller, std::string LocationName);
/// New at v1.2.0; true if a track element present (not inactive elements - see InactiveTrackElementPresentAtHV
    bool trackElementPresentAtHV(int caller, int h_loc, int v_loc);
/// New at v1.2.0; checks whether a train present at input location and link and returns its ID if so
    bool trainOnLink(int caller, int h_loc, int v_loc, int link, int &trainid);
/// Handles all tasks associated with track linking, returns true if successful (see also LinkTrack & LinkTrackNoMessages above)
    bool tryToConnectTrack(int caller, bool &locerror, int &h_loc, int &v_loc, bool giveMessages);
/// Return a pointer to the point fillet (the bit that appears to move when points are changed) for the point and its Attribute specified in TrackElement
    //Graphics::TBitmap *GetFilletGraphic(int caller, TTrackElement TrackElement);
/// Return a pointer to the striped (i.e. when unnamed) graphic corresponding to TrackElement, if TrackElement isn't a named element just return its normal graphic
    //Graphics::TBitmap *RetrieveStripedNamedLocationGraphicsWhereRelevant(int caller, TTrackElement TrackElement);
/// Return the link array position for the element at StartTVPosition that gives the closest link to the element a EndTVPosition. NB the StartTVPosition is expected to be a single track element as only positions 0 & 1 are checked
    int findClosestLinkPosition(int caller, int starttvposition, int endtvposition);
/// Return the opposite link position for the element at TrackVectorPosition with link position LinkPos, if the element is points and they are set against the exit then &Derail is returned true
    int getAnyElementOppositeLinkPos(int caller, int trackvectorposition, int linkpos, bool &Derail);
/// Takes the ElementID value (an std::string) (e.g. "8-13", "N43-N127", etc) and returns the corresponding track vector position, if none is found then -1 is returned
    int getTrackVectorPositionFromString(int caller, std::string String, bool giveMessages);
/// Returns the track vector position corresponding to the h_loc & v_loc positions, FoundFlag indicates whether an element is found or not, and if not -1 is returned
    int getVectorPositionFromTrackMap(int caller, int h_loc, int v_loc, bool &FoundFlag);
/// Returns the number of gaps in the railway
    int numberOfGaps(int caller);
/// Returns the number of separate platforms (not platform elements) at a given location, either a station or non station named location
    int numberOfPlatforms(int caller, std::string LocationName);
/// Similar to GetVectorPositionFromTrackMap but for inactive elements, a pair is returned because there can be up to 2 platforms at a specific position. If nothing found each element of pair is 0 & FoundFlag false
    IMPair getVectorPositionsFromInactiveTrackMap(int caller, int h_loc, int v_loc, bool &FoundFlag);
/// Searches LocationNameMultiMap to check if the element pointed to by the TTrackVectorIterator has the name LocationName. If it finds it the pointer TLocationNameMultiMapIterator is returned.  If it fails ErrorString is set to an appropriate text to allow the calling function to report the error.  Otherwise it is set to "".
    LocationNameMultiMapIterator findNamedElementInLocationNameMultiMap(int caller, std::string LocationName, TrackVectorIterator TrackElement,
                                                                         std::string &ErrorString);
/// Return a reference to the inactive element at h_loc & v_loc, if no element is found an error is thrown
    TrackElement &getInactiveTrackElementFromTrackMap(int caller, int h_loc, int v_loc);
/// Return a reference to the element at h_loc & v_loc for any map and any vector (used for SelectPrefDir in clipboard pref dir recovery
    TrackElement &getTrackElementFromAnyTrackMap(int caller, int h_loc, int v_loc, TrackMap &map, TrackVector &vector); //new at v2.9.0 for clipboard pref dirs & modified at v2.9.2 to make Map & Vector references
/// Return a reference to the element at h_loc & v_loc, if no element is found an error is thrown
    TrackElement &getTrackElementFromTrackMap(int caller, int h_loc, int v_loc);
/// A range-checked version of InactiveTrackVector.at(at)
    TrackElement &inactiveTrackElementAt(int caller, int at);
/// A range-checked version of SelectVector.at(at)
    TrackElement &selectVectorAt(int caller, int at);
/// A range-checked version of TrackVector.at(at)
    TrackElement &trackElementAt(int caller, int at);
/// Takes an adjusted vector position value from either vector (if active, Position = -TruePos -1, if inactive, Position = TruePos) and returns a pointer to the relevant element
    TrackVectorIterator getTrackVectorIteratorFromNamePosition(int caller, int position);
/// A range-checked version of UserGraphicVector.at(at)
    //UserGraphicItem &UserGraphicVectorAt(int caller, int at);
/// TrackElement.LocationName becomes 'Name' (for active and inactive elements) and, if TrackElement is a platform or named non-station location, any active element at the same h_loc & v_loc position has its ActiveTrackElementName set to 'Name'.
    void addName(int caller, TrackVectorIterator track_element, std::string name);
/// Examine TrackVector and whenever find a new gap pair enter it into GapMap
    void buildGapMapFromTrackVector(int caller);
/// Examine TrackVector, InactiveTrackVector and TextVector, and set the values that indicate the extent of the railway (h_locMin, v_locMin, h_locMax & v_locMax) for use in zoomed in and out displays and in saving railway images
    void calch_locMinEtc(int caller);
/// Changes the LocationName in the name multimap to NewName at the location pointed to by the TLocationNameMultiMapIterator from whatever it was before.  Accepts null entries so that a formerly named element can have the name changed to "".
    void changeLocationNameMultiMapEntry(int caller, std::string new_name, LocationNameMultiMapIterator sn_iterator);
/// Validity test
    void checkGapMap(int caller);
/// Validity test
    void checkLocationNameMultiMap(int caller);
/// Validity test
    void checkMapAndInactiveTrack(int caller);
/// Validity test
    void checkMapAndTrack(int caller);
/// After an element has been erased from the TrackVector, all the later elements are moved down one.  This function decrements the position values for all values above that of the erased element in the gap elements, TrackMap and LocationNameMultiMap.
    void decrementValuesInGapsAndTrackAndNameMaps(int caller, unsigned int vec_pos);
/// After an element has been erased from the InactiveTrackVector, all the later elements are moved down one.  This function decrements the position values for all values above that of the erased element in both InactiveTrack2MultiMap and LocationNameMultiMap.
    void decrementValuesInInactiveTrackAndNameMaps(int caller, unsigned int vec_pos);
/// All platform, concourse, footcrossing & non-station named location elements are able to have a LocationName allocated.
/**Track elements (including footcrossings) are able to have an ActiveTrackElementName allocated provided there is an adjacent platform or a NamedNonStationLocation.
To set these names the user selects a single named location element (not a footcrossing), enters the name, and this is then allocated as a LocationName
to all linked platform, concourse and footcrossing elements, and as an ActiveTrackElementName to all track elements adjacent to
platforms (inc footcrossing tracks if (but only if) they have a platform at that location). */
    void enterLocationName(int caller, std::string location_name, bool adding_elements);
/// Examines LocationNameMultiMap and if the LocationName is found all elements at that H & V (in both active and inactive vectors) have the name erased both as a LocationName and a ActiveTrackElementName.  The LocationNameMultiMap is also rebuilt to correspond to the new names in the vectors.
    void eraseLocationAndActiveTrackElementNames(int caller, std::string location_name);
/// Erases all active and inactive track elements at h_loc_input & v_loc_input from the vectors, and, if any of these elements are named the entries are erased from LocationNameMultiMap and the corresponding name is removed from the display and from all other linked named elements
    void eraseTrackElement(int caller, int h_loc_input, int v_loc_input, int &erased_track_vector_position, bool &track_erase_successful, bool internal_checks);
/// With large railways only part of the railway is displayed on screen, and this function converts true (relative to the whole railway) H & V positions to screen (relative to the displayed screen) H & V positions
    void getScreenPositionsFromTruePos(int caller, int &screen_pos_h, int &screen_pos_v, int h_pos_true, int v_pos_true);
/// Converse of GetScreenPositionsFromTruePos except that in this function h_loc & v_loc are expressed in track elements (i.e. 16x16 pixel squares) rather than in single pixels
    void getTrackLocsFromScreenPos(int caller, int &h_loc, int &v_loc, int screen_pos_h, int screen_pos_v);
/// Converse of GetScreenPositionsFromTruePos
    void getTruePositionsFromScreenPos(int caller, int &h_pos, int &v_pos, int screen_pos_h, int screen_pos_v);
/// Examine all elements in the TrackVector and if have a valid length mark the relevant track using MarkOneLength
    //void lengthMarker(int caller, TDisplay *Disp);
/// Load all BarriersDownVector values from SessionFile
    void loadBarriersDownVector(int caller, std::ifstream &vec_instream);
/// new at v2.4.0, load user graphics
    void loadGraphics(int caller, std::ifstream &vec_instream, std::filesystem::path &graphics_path);
/// Load track elements (active & inactive) from the file into the relevant vectors and maps, and try to link the resulting track
    void loadTrack(int caller, std::ifstream &vec_instream, bool &graphics_follow);
/// Mark on screen a track element according to its length and speed limit if either of these differ from their default values
    //void markOneLength(int caller, TrackElement TE, bool first_track, TDisplay *Disp);
/// Called during track building or pasting, when an element identified by CurrentTag (i.e. its SpeedTag value) is to be placed at position h_loc_input & v_loc_input.  If the element can be placed it is displayed and added to the relevant vector, and if named its name is added to LocationNameMultiMap. At v2.2.0 'Aspect' added so can distinguish between adding and pasting track
    void plotAndAddTrackElement(int caller, int current_tag, int aspect, int h_loc_input, int v_loc_input, bool &track_plotted, bool internal_checks);
/// Plots a continuation on screen, may have overlays if a multiplayer session
    //void plotContinuation(int caller, TrackElement track_element, TDisplay *Disp);
///new at v2.2.0 - as PlotAndAddTrackElement but keeping speed & length attributes (for pasting) and also pasting location names
    void plotPastedTrackElementWithAttributes(int caller, TrackElement temp_track_element, int h_loc_input, int v_loc_input, bool &track_linking_required,
                                              bool internal_checks);
/// Just replot the basic track elements at a level crossing (for flashing)
    //void plotLCBaseElementsOnly(int caller, barrierstate state, int base_element_speed_tag, int h_loc, int v_loc, int type_of_route, TDisplay *disp);
/// Plot & open (to trains) all level crossings linked to TrackElement (Manual true = manually lowered, colour is green)
	//void plotLoweredLinkedLevelCrossingBarriers(int caller, int base_element_speed_tag, int h_loc, int v_loc, int type_of_route, TDisplay *Disp, bool manual);
/// Plot LC elements without any base elements, and set LCPlotted true - used in ClearandRebuildRailway (manual true = manually lowered, colour is green)
    //void plotPlainLoweredLinkedLevelCrossingBarriersAndSetMarkers(int caller, int base_element_speed_tag, int h_loc, int v_loc, TDisplay *Disp, bool manual);
/// Plot LC elements without any base elements, and set LCPlotted true - used in ClearandRebuildRailway
    //void plotPlainRaisedLinkedLevelCrossingBarriersAndSetMarkers(int caller, int base_element_speed_tag, int h_loc, int v_loc, TDisplay *disp);
/// Plot & close (to trains) all level crossings linked to TrackElement - always plots as red - auto
    //void plotRaisedLinkedLevelCrossingBarriers(int caller, int base_element_speed_tag, int h_loc, int v_loc, TDisplay *disp);
/// Plots either a LC or a blank element to flash manual LCs in zoomout mode
    //void plotSmallFlashingLinkedLevelCrossings(int caller, int h_loc, int v_loc, Graphics::TBitmap *graphic_ptr, TDisplay *disp);
/// Plots a gap on screen - may be set or unset
    //void plotGap(int caller, TrackElement TrackElement, TDisplay *Disp);
/// Plot points on screen according to how they are set (attribute value), or, with both fillets if BothFillets is true (the fillet is the bit that appears to move when points are changed)
    //void plotPoints(int caller, TrackElement TrackElement, TDisplay *Disp, bool bothFillets);
/// Plot signals on screen according to their aspect (attribute value)
    //void plotSignal(int caller, TTrackElement TrackElement, TDisplay *Disp);
/// Plot platforms if any for a signal graphic - plotted before signal so shows through transparent signal pixels
    //void plotSignalPlatforms(int caller, int h_loc, int v_loc, TDisplay *Disp);
/// Plot on screen the zoomed-out railway
    //void plotSmallRailway(int caller, TDisplay *Disp);
/// Plot on screen in zoomed-out mode and in gap setting mode a small red square corresponding to the gap position that is waiting to have its matching gap selected (see also ShowSelectedGap)
    void plotSmallRedGap(int caller);
/// Add all LCs to LCVector - note that this contains all LC elements whether linked to others or not
    void populateLCVector(int caller);
/// clear then add all simple element track vector positions to the vector, added at v2.13.0
    void populateSimpleVector(int caller);
/// Clears the existing LocationNameMultiMap and rebuilds it from TrackVector and InactiveTrackVector. Called after the track is linked as many of the vector positions are likely to change - called from RepositionAndMapTrack(); after names are changed in EraseLocationAndActiveTrackElementNames; and after the name changes in EnterLocationName.
    void rebuildLocationNameMultiMap(int caller);
/// Called by TInterface::ClearandRebuildRailway to replot all the active and inactive track elements and text, BothPointFillets indicates whether points are to be plotted according to how they are set - for operation, or with both fillets - when not operating or during development (the fillet is the bit that appears to move when points are changed)
    //void rebuildTrackAndText(int caller, TDisplay *Disp, bool bothPointFilletsAndBasicLCs);
/// rebuild user graphics
   // void rebuildUserGraphics(int caller, TDisplay *Disp);
/// restore points to unfailed state, added at v2.13.0
    void repairFailedPoints(FailedElementVector::iterator fpv_it);
/// restore signal to unfailed state, added at v2.13.0
    void repairFailedSignals(FailedElementVector::iterator fpv_it);
/// remove TSR, added at v2.13.0
    void repairTSR(FailedElementVector::iterator fpv_it);
/// Track elements have members that indicate whether and on what track a train is present (trainIDOnElement, TrainIDOnBridgeOrFailedPointOrigSpeedLimit01 and TrainIDOnBridgeOrFailedPointOrigSpeedLimit23).  This function resets them all to their non-train-present state of -1. Called by TTrainController::UnplotTrains
/// Also used for failed points to store original speedlimits
    void resetAllTrainIDsAndFailedPointOrigSpeedLimits(int caller);
/// Called by EraseTrackElement after the element has been erased and the vector positions changed, in order to reset a matching gaps if the erased element was a set gap
    void resetAnyNonMatchingGaps(int caller);
/// Set all LC attributes to 0 (closed to trains)
    void resetLevelCrossings(int caller);
/// Called on exit from operation to reset all points to non-diverging or to left fork (attribute = 0)
    void resetPoints(int caller);
/// Called on exit from operation to reset all signals to red (attribute = 0)
    void resetSignals(int caller);
/// Save all changing vector values (used for error file)
    void saveChangingLCVector(int caller, std::ofstream &out_stream);
/// Save all vector values to the session file
    void saveSessionBarriersDownVector(int caller, std::ofstream &out_stream);
/// Save all active and inactive track elements to VecFile
    void saveTrack(int caller, std::ofstream& vec_stream, bool grapics_follow);
/// save graphics
    void saveUserGraphics(int caller, std::ofstream &vec_stream);
/// Checks all locations that are adjacent to the one entered for linked named location elements.
/**If any LocationName is found in any of the linked elements, that name is used for all elements that are now linked to it. The location entered doesn't need
to be a FixedNamedLocationElement and there doesn't even need to be an element there.
Used during EraseTrackElement (in which case the SpeedTag is that for the element that is erased) and PlotAndAddTrackElement,
to bring the named location and timetable naming up to date with the deletion or insertion.*/
    void searchForAndUpdateLocationName(int caller, int h_loc, int v_loc, int speed_tag);
/// Work through all elements in TrackVector setting all lengths & speed limits to default values - including both tracks for 2-track elements
    void setAllDefaultLengthsAndSpeedLimits(int caller);
/// Set TypeOfRoute value to 2 to indicate barriers manually closed
    void setBarriersDownLCToManual(int caller, int h_loc, int v_loc);
/// Convert the position values for the TrackElement into an identification string and load in ElementID
    void setElementID(int caller, TrackElement &track_element);
/// Set LC attribute at H & V; 0=closed to trains, 1 = open to trains, 2 = changing state = closed to trains
    void setLCAttributeAtHV(int caller, int h_loc, int v_loc, int attr);
/// Set linked LC attributes; 0=closed to trains, 1 = open to trains, 2 = changing state = closed to trains
    void setLinkedLevelCrossingBarrierAttributes(int caller, int h_loc, int v_loc, int attr);
/// Set all TypeOfRoute values to 2 for all linked LCs to indicate manually lowered
    void setLinkedManualLCs(int caller, int h_loc, int v_loc);
/// Called when trying to link track and when a name changed when track already linked.
/**Examines all track elements that have ActiveTrackElementName set, sums the number of consecutive elements with the same name,
and sets the EntryLink values for the front of train stop points for each direction.  For stations (not non-station named
locations) of length n, where n > 1, stop element is [floor((n+1)/2) + 1] from each end (unless buffers at one or both ends in
which case stop points are the end elements).  Note that for a single element the stop point is the element itself (formula
doesn't apply).  For NamedNonStationLocations the stop points are at the end elements to allow trains to stack up. */
    void setStationEntryStopLinkPosses(int caller);
/// Called during gap setting to mark a gap with a red circle - after which the program awaits user selection of the matching gap
    //void showSelectedGap(int caller, TDisplay *Disp);
/// Empty the track and inactive track vectors, the corresponding track maps, and LocationNameMultiMap
    void trackClear(int caller);
/// Insert TrackElement into the relevant vector and map, and, if named, insert the name in LocationNameMultiMap
    void trackPush(int caller, TrackElement track_element);
/// handles moving of user graphics
    void userGraphicMove(int caller, int h_pos_input, int v_pos_input, int &user_graphics_item, int &user_graphic_move_h_pos, int &user_graphic_move_v_pos,
                         bool &user_graphics_found);
/// Called by TInterface::SaveOperatingImage1Click to add all track & text to the image file in their operating state
    //void writeOperatingTrackAndTextToImage(int caller, Graphics::TBitmap *Bitmap);
/// Called by SaveImageNoGridMenuItemClick, SaveImageAndGridMenuItemClick amd SaveImageAndPrefDirsMenuItemClick to add all track element graphics to the image file in their non-operating state
    //void writeGraphicsToImage(int caller, Graphics::TBitmap *Bitmap);
/// Called by TInterface::SaveImageNoGrid1Click, TInterface::SaveImageAndGrid1Click and TInterface::SaveImageAndPrefDirs1Click to add all track & text to the image file in the non-operating state
    //void writeTrackAndTextToImage(int caller, Graphics::TBitmap *Bitmap);

/// Constructor, only one object of this class
    Track();
/// Destructor
    ~Track();
};


/// A descendent of TOnePrefDir used for routes.  Used during contruction of a route (ConstructRoute) and also for all completed routes, when each route is saved as an entry in the AllRoutesVector (see TAllRoutes)
class OneRoute : public OnePrefDir
{
public:
/// A single flashing element of a route that flashes during setting
    class RouteFlashElement
    {
    public:
        int h_loc, v_Loc, track_vector_pos;
///< element values
        // Graphics::TBitmap *OriginalGraphic, *OverlayGraphic;
///< the two graphics, non route-coloured and route-coloured respectively, these are
        ///< displayed alternately during flashing
    };

/// The flashing route
    class RouteFlash
    {
    public:
        std::vector<RouteFlashElement>route_flash_vector;
        bool overlay_plotted{false};
///< flag indicating the graphic that is currently displayed, true for the overlay (route-coloured)

// both external
        void plotRouteOverlay(int caller);
///< display the overlay (route-coloured) graphic
        void plotRouteOriginal(int caller);
///< display the original (non route-coloured) graphic
    };

    static const int route_search_limit{30000};
///< limit to the number of elements searched in attempting to find a route

    IDInt req_pos_route_id{0};
///< the route ID number of the route that is being extended backwards during route building, not needed for
    ///< session saves as routes in build are not saved in sessions
    IDInt start_selection_route_id{0};
///< the route ID number of the route that is being extended forwards during route building, not
    ///< needed for session saves as routes in build are not saved in sessions

    int route_id{0};
///< the ID number of the route, this is needed for session saves
    int start_route_pos{0};
///< TrackVectorPosition of the StartElement(s) set when the starting position of a new route is selected, note that although there may be two StartElements (as there can be two preferred directions on a single element), there is only one TrackVectorPosition as the element is the same for both
    PrefDirElement start_element, start_element_2;
///< the two preferred direction elements corresponding to the starting position of a new route
    RouteFlash route_flash;
///< the class member that allows the route to flash during setting up (see TRouteFlash above)

// inline functions

/// Empty the route of any stored elements
    void clearRoute()
    {
        pref_dir_vec.clear();
        search_vec.clear();
    }

/// Erase a single route element
    void eraseRouteElementAt(std::vector<PrefDirElement>::iterator route_element_ptr)
    {
        pref_dir_vec.erase(route_element_ptr);
    }

/// Store a single route element in the PrefDirVector
    void storeRouteElementInPrefDirVector(PrefDirElement load_ref_dir_element)
    {
        load_ref_dir_element.is_a_route = true;
        pref_dir_vec.push_back(load_ref_dir_element);
    }

// functions defined in .cpp file

/// Used when setting signal aspects for a route by working forwards through the route to see what the next forward signal aspects is, because this determines all the rearward signal aspects.
    bool findForwardTargetSignalAttribute(int caller, int &next_forward_linked_route_number, int &attribute) const;
/// Set the starting conditions for a non-preferred (i.e. unrestricted) route selection beginning on h_loc & v_loc
    bool getNonPreferredRouteStartElement(int caller, int h_loc, int v_loc, bool call_on);
/// Try to find a set of linked tracks between the route start element and the one at h_loc & v_loc.  If find one return true, set &PointsChanged to true if any points need to be changed and &ReqPosRouteID to the route ID of the existing route to attach to, if there is one, and -1 if not
    bool getNextNonPreferredRouteElement(int caller, int h_loc, int v_loc, bool call_on, IDInt &req_pos_route_id, bool &points_changed);
/// Set the starting conditions for a preferred direction or automatic signal route selection beginning on h_loc & v_loc
    bool getPreferredRouteStartElement(int caller, int h_loc, int v_loc, std::shared_ptr<OnePrefDir> every_pref_dir, bool auto_sigs_flag);
/// Try to find a set of linked tracks that lie on preferred directions between the route start element and the one at h_loc & v_loc.  If find one return true, set &PointsChanged to true if any points need to be changed and &ReqPosRouteID to the route ID of the existing route to attach to, if there is one, and -1 if not
    bool getNextPreferredRouteElement(int caller, int h_loc, int v_loc, std::shared_ptr<OnePrefDir> every_pref_dir, bool consec_signals, bool auto_sigs_flag,
                                      IDInt &req_pos_route_id, bool &points_changed);
/// Called by GetNextNonPreferredRouteElement and GetNextPreferredRouteElement to check whether or not any points on the selected route need to be changed.
/// Also gives every set of points that needs to change the chance to fail, and if so returns the TV position, added at v2.13.0
    bool pointsToBeChanged(int caller, int &new_failed_points_tv_pos) const;
/// Called by GetNextNonPreferredRouteElement to carry out the search for linked track, and also called recursively, if recursive RecursiveCall is true (added at v2.13.0)
    bool searchForNonPreferredRoute(int caller, TrackElement current_track_element, int x_link_pos, int required_pos, IDInt req_pos_route_id, bool recursive_call);
/// Called by GetNextPreferredRouteElement to carry out the search for a valid route, and also called recursively, if recursive RecursiveCall is true (added at v2.13.0)
    bool searchForPreferredRoute(int caller, PrefDirElement pref_dir_element, int x_link_pos, int required_pos, IDInt req_pos_route_id, std::shared_ptr<OnePrefDir> every_pref_dir,
                                 bool consec_signals, int end_select_pos, bool auto_sigs_flag, bool recursive_call);
/// Called by TAllRoutes::SetAllRearwardsSignals to set rearwards signals from a specified starting position.  If a train is found during the rearwards search then this function flags the fact so that the calling function can change its behaviour with respect to further rearwards signal aspects.
    bool setRearwardsSignalsReturnFalseForTrain(int caller, int &attribute, int pref_dir_vector_start_pos) const;
/// Check incorporated in route search routines after have found a legitimate route, returns false for signal failure & deals with graphics & messages
    bool signalHasFailed(int caller); //added at v2.13.0
/// Called after a non-preferred (i.e. unrestricted) route has been selected and has finished flashing, to add it to the AllRoutesVector
    void convertAndAddNonPreferredRouteSearchVector(int caller, IDInt req_pos_route_id);
/// Called after a preferred (i.e. preferred direction or automatic signals) route has been selected and has finished flashing, to add it to the AllRoutesVector
    void convertAndAddPreferredRouteSearchVector(int caller, IDInt req_pos_route_id, bool auto_sigs_flag);
/// Cancel a route immediately if a train occupies it when travelling in the wrong direction (or occupies a crossover on a non-route line when the other track is in a route)
    void forceCancelRoute(int caller);
/// Examines the route to see whether the element at H & V is in the route, and if not returns a ReturnFlag value of NotInRoute.
/** If it is in a route but the element selected is invalid, then a message is given and returns with a ReturnFlag value of
InRouteFalse.  Otherwise the route is truncated at and including the element that matches H & V with a ReturnFlag value of
InRouteTrue.  Selection invalid if select a bridge; trying to leave a single element; last
element to be left not a signal (for PrefDirRoute or AutoSigsFlag set); last element to be left a bridge, points or crossover
(for not PrefDirRoute & AutoSigsFlag not set), or part of route locked. */
    void getRouteTruncateElement(int caller, int h_loc, int v_loc, bool PrefDirRoute, TruncateReturnType &return_flag);
/// Used when creating a bitmap image to display the route colours and direction arrows (as on screen during operation) for an operating railway
    // void RouteImageMarker(int caller, Graphics::TBitmap *Bitmap) const;
/// After a route has been selected successfully this function sets all LC change values appropriately for the selected route type and location
    void setLCChangeValues(int caller, bool pref_dir_route);
/// Called when setting unrestricted routes to set the route element values appropriately after a successful search has been conducted.  It isn't needed for preferred routes because the element values are obtained from the already set preferred direction elements
    void setRemainingSearchVectorValues(int caller);
/// After a route has been selected successfully this function sets all RouteFlash (see above) values appropriately for the selected route type and location
    void setRouteFlashValues(int caller, bool auto_sigs_flag, bool pref_dir_route);
/// Set values for EXGraphicPtr and EntryDirectionGraphicPtr for all elements in SearchVector so that the route displays with the correct colour
    void setRouteSearchVectorGraphics(int caller, bool auto_sigs_flag, bool pref_dir_route);
/// Called when setting a route to set all points appropriately
    void setRoutePoints(int caller) const;
/// Called when setting a route to set all signals appropriately.  Also called when a new train is added at a position where a route has been set, when it is necessary to set the next rearwards signal to red, the next yellow etc.
    void setRouteSignals(int caller) const;
};

/// Handles data and functions relating to all routes on the railway
class AllRoutes
{
public:

/// Handles routes that are locked because of approaching trains
    class LockedRouteClass
    {
    public:
        int route_number;
///< the vector position number of the relevant route in AllRoutesVector
        unsigned int truncate_track_vector_pos;
///< the TrackVector position of the element selected for truncation
        unsigned int last_track_vector_pos;
///< the TrackVector position of the last (i.e. most forward) element in the route
        int last_xlink_pos;
///< the XLinkPos value of the last (i.e. most forward) element in the route
        boost::posix_time::time_duration lock_start_time;
///< the timetable time at which the route is locked, to start the 2 minute clock
    };

    enum class RouteType
    {
        NoRoute, NotAutoSigsRoute, AutoSigsRoute
    } route_type;
///< distinguishes between automatic signals routes and other types, or no route at all (where this is used there is no need to distinguish between preferred direction and unrestricted routes)

    typedef std::vector<OneRoute>AllRoutesVector;
///< the vector class that holds all the railway routes
    typedef std::vector<OneRoute>::iterator AllRoutesVectorIterator;

    typedef std::vector<LockedRouteClass>LockedRouteVector;
///< the vector class that holds all locked routes
    typedef std::vector<LockedRouteClass>::iterator LockedRouteVectorIterator;

    typedef std::pair<int, unsigned int> RouteElementPair;
///< defines a specific element in a route, the first (int) value is the vector position in the AllRoutesVector, and the second (unsigned int) value is the vector position of the element in the route's PrefDirVector
    typedef std::multimap<HVPair,RouteElementPair, MapComp>Route2MultiMap;
///< the multimap class holding the elements of all routes in the railway.  The first entry is the h_loc & v_loc pair values of the route element, and the second is the TRouteElementPair defining the element.  There are a maximum of 2 elements per h_loc & v_loc location
    typedef Route2MultiMap::iterator Route2MultiMapIterator;
    typedef std::pair<HVPair, RouteElementPair>Route2MultiMapEntry;

/// Used to store relevant values when a call-on found, ready for plotting an unrestricted route
    class CallOnEntry
    {
    public:
        bool route_or_part_route_set;
///< whether or not a route or part route already plotted
        int route_start_pos;
///< the stop signal trackvectorposition
        int platform_pos;
///< the first platform trackvectorposition

/// Constructor
        CallOnEntry(bool route_or_part_route_set_ip, int route_start_position_ip, int platform_position_ip)
        {
            route_or_part_route_set = route_or_part_route_set_ip;
            route_start_pos = route_start_position_ip;
            platform_pos = platform_position_ip;
        }
    };

    std::vector<CallOnEntry>call_on_vector;
///< the store of all call-on entries

    bool locked_route_found_during_route_building{false};
///< this flags the fact that a locked route has been found during route building in an existing linked route which is erased prior to its elements being added to the new route.

// the following variables store the locked route values for reinstating after a locked route has been found during route building in an
// existing linked route which is erased prior to its elements being added to the new route.  The locked route is erased in
// ClearRouteDuringRouteBuildingAt, and is reinstated in ConvertAndAddPreferredRouteSearchVector or ConvertAndAddNonPreferredRouteSearchVector.
    int locked_route_last_x_link_pos{0};
    unsigned int locked_route_truncate_track_vector_pos{0};
    unsigned int locked_routed_last_track_vector_pos{0};
    boost::posix_time::time_duration locked_route_lock_start_time{0, 0, 0, 0};
// end of locked route values

    bool rebuild_railway_flag{false};
///< this is set whenever a route has to be cancelled forcibly in order to force a ClearandRebuildRailway at the next clock tick if not in zoom-out mode to clear the now cancelled route on the display
    bool route_truncate_flag{false};
///< used to flag the fact that a route is being truncated on order to change the behaviour of signal aspect setting in SetRearwardsSignalsReturnFalseForTrain

    const float level_crossing_barrier_up_delay{10.};
///< the full value in seconds for which the level crossing flashes prior to closing to trains
    const float level_crossing_barrier_down_delay{30.};
///< the full value in seconds for which the level crossing flashes prior to opening to trains
    const float points_delay{2.5};
///< the value in seconds for which points flash prior to being changed.  Used for the points flash period when changing points manually and for the route flash period when points have to be changed
    const float signals_delay{0.5};
///< the value in seconds for which signals flash prior to being changed.  Used for the route flash period when points don't have to be changed
    int next_route_id{0};
///< stores the value for the route ID number that is next to be built
    AllRoutesVector all_routes_vector;
///< the vector that stores all the routes on the railway
    LockedRouteVector locked_route_vector;
///< the vector that stores all the locked routes on the railway
    OneRoute siganller_removed_train_auto_route;
///< if train was on an AutoSigsRoute when removed then this stores the route so that signals can be reset
    Route2MultiMap route_2_mm;
///< the map that stores the elements of all routes on the railway (see TRoute2MultiMap for more info)

// inline functions

/// Returns the number of routes in the railway
    unsigned int allRoutesSize() const
    {
        return all_routes_vector.size();
    }

/// Erases all routes from AllRoutesVector and from Route2MultiMap
    void allRoutesClear()
    {
        all_routes_vector.clear();
        route_2_mm.clear();
    }
/// Functions defined in .cpp file
    bool checkForLoopingRoute(int caller, int end_position, int end_x_link_pos, int start_position); // return true if route loops back on itself
/// Performs an integrity check on the routes stored in a session file and returns false if there is an error
    bool checkRoutes(int caller, int number_of_active_elements, std::ifstream &in_stream);
/// The track geometry allows diagonals to cross without occupying the same track element, so when route plotting it is necessary to check if there is an existing route or a train on such a crossing diagonal.  Returns true for a fouled (i.e. fouled by a route or a train) diagonal. New at v1.2.0
    bool diagonalFouledByRouteOrTrain(int caller, int h_loc, int v_loc, int diagonal_link_number);
/// As above but only checks for a route (may or may not be a train present (new at v1.2.0)
    bool diagonalFouledByRoute(int caller, int h_loc, int v_loc, int diagonal_link_number);
/// If a route is present at H, V & Elink returns true with RouteNumber giving vector position in AllRoutes vector.  Returns false for anything else including no element or route at H & V etc. New at v1.2.0
    bool findRouteNumberFromRoute2MultiMapNoErrors(int caller, int h_loc, int v_loc, int e_link, int &route_number);
/// Examines all routes and for each uses GetRouteTruncateElement to see if the element at H & V is present in that route.
/**  The ReturnFlag value indicates InRouteTrue (success), InRouteFalse (failure), or NotInRoute.  Messages are given in GetRouteTruncateElement.
If successful the route is truncated at and including the element that matches H & V.  If PrefDirRoute ensure only truncate to a signal, else prevent
truncation to a crossover, bridge or points, also prevent route being left less than 2 elements in length.*/
    bool getAllRoutesTruncateElement(int caller, int h_loc, int v_loc, bool pref_dir_route);
/// Checks whether the preferred direction element at TrackVectorPosition with XLinkPos value is in a locked route and returns true if so together with the element itself copied to &PrefDirElement & the LockedRouteVector position in &LockedVectorNumber
    bool isElementInLockedRouteGetPrefDirElementGetLockedVectorNumber(int caller, int track_vector_position, int x_link_pos, PrefDirElement &pref_dir_element,
                                                                      int &locked_vector_number);
/// Returns true if there is a route with the given ID number - added at v1.3.1 (see function for details)
    bool isThereARouteAtIDNumber(int caller, IDInt route_id);
/// Loads the routes from a session file
    bool loadRoutes(int caller, std::ifstream &in_stream);
/// Route locking is required (returns true) if a moving train is within 3 signals back from the RouteTruncatePosition (on the route itself or on any linked routes, or on the element immediately before the start of the route or linked route - this because train cancels route elements that it touches) unless the first signal is red, then OK
    bool routeLockingRequired(int caller, int route_number, int route_truncate_position);
/// Examines Route2MultiMap and if the element at TrackVectorPosition with LinkPos (can be entry or exit) is found it returns true (for crossovers & points returns true whichever track the route is on), else returns false.
    bool trackIsInARoute(int caller, int track_vector_position, int link_pos);
/// Returns a route's position in AllRoutesVector from its ID, throws an error if a matching route isn't found
    int getRouteVectorNumber(int caller, IDInt route_id);
/// Returns a constant reference to the route at AllRoutesVector position 'At', after performing range checking on the 'At' value and throwing an error if out of range
    const OneRoute &getFixedRouteAt(int caller, int at) const;
/// Returns a constant reference to the route with ID number RouteID.  If no route is found with that ID an error is thrown
    const OneRoute &getFixedRouteAtIDNumber(int caller, IDInt route_id) const;
/// Returns a modifiable reference to the route at AllRoutesVector position 'At', after performing range checking on the 'At' value and throwing an error if out of range
    OneRoute &getModifiableRouteAt(int caller, int at);
/// Returns a modifiable reference to the route with ID number RouteID. If no route is found with that ID an error is thrown
    OneRoute &getModifiableRouteAtIDNumber(int caller, IDInt route_id);
/// Examines Route2MultiMap and returns a TRouteElementPair if one is found with the passed values of H, V and ELink.
/**Also returned as a reference is an iterator to the found element in the map to assist in erasing it.  Called by
TAllRoutes::RemoveRouteElement.  Note that only need ELink (as well as H & V) to identify uniquely, since only bridges can have
two routes on them & their track ELinks are always different.  Messages are given for failure.*/
    RouteElementPair findRoutePairFromRoute2MultiMap(int caller, int h_loc, int v_loc, int e_link, Route2MultiMapIterator &route_2_mm_iter);
/// Retrieve up to two TRouteElementPair entries from Route2MultiMap at H & V, the first as a function return and the second in the reference SecondPair.  If there's only one then it's the function return
    RouteElementPair getRouteElementDataFromRoute2MultiMap(int caller, int h_loc, int v_loc, RouteElementPair &second_pair);
/// Examines Route2MultiMap for the element at TrackVectorPosition with LinkPos (can be entry or exit).
/**Returns the appropriate route type - NoRoute, NotAutoSigsRoute, or AutoSigsRoute.  If element is in a
route then the EXGraphicPtr is returned, and if either the start or end of a route then the correct EntryDirectionGraphicPtr is
returned, else a transparent element is returned.  Function is used int trainUnit for retaining AutoSigsRoutes but erasing others
after train passes, and for picking up the correct background graphics for replotting of AutoSigsRoutes; also used in
CallingOnAllowed*/
    // RouteType getRouteTypeAndGraphics(int caller, int trackvectorposition, int linkpos, Graphics::TBitmap* &EXGraphicPtr,
    //                                    Graphics::TBitmap* &EntryDirectionGraphicPtr);
/// Examines Route2MultiMap and if the element at TrackVectorPosition with LinkPos (can be entry or exit) is found returns the appropriate route type - NoRoute, NotAutoSigsRoute, or AutoSigsRoute and number (i.e. its position in AllRoutesVector, -1 if NoRoute).
    RouteType getRouteTypeAndNumber(int caller, int track_vector_position, int link_pos, int &route_number);
/// A single TPrefDirElement is added to both PrefDirVector (for the route at RouteNumber) and Route2MultiMap.  Called from TAllRoutes::StoreOneRoute.  Note that the IsARoute boolean variable is set in StoreRouteElementInPrefDirVector since that catches all route elements wherever created
    void addRouteElement(int caller, int h_loc, int v_loc, int elink, int routenumber, PrefDirElement RouteElement);
/// Diagnostic function - checks equivalence for each route between entries in PrefDirVector and those in Route2MultiMap, and also that the size of the multimap and the sum of the sizes of all PrefDirVectors is the same.  Throws an error if there is a discrepancy.
    void checkMapAndRoutes(int caller);
/// When attaching a new route section to an existing route, it is sometimes necessary to erase the original route and create a new composite route.
/**This function erases all elements in the route at RouteNumber using TAllRoutes->RemoveRouteElement to clear elements from Route2MultiMap
and from the PrefDirVector.  Since all elements for the route are removed RemoveRouteElement also clears the Route from AllRoutesVector.
Route numbers are decremented in the map for route numbers that are greater than the route number that is removed.  The LockedRouteVector
as also searched and if any relate to the route that has been cleared they are erased too, but the fact that one has been found is recorded
so that it can be re-established later.*/
    void clearRouteDuringRouteBuildingAt(int caller, int route_number);
/// After a route element has been erased from the relevant PrefDirVector and from Route2MultiMap, this function examines all the remaining entries in Route2MultiMap with the same RouteNumber as that for the erased element.  Where a RouteElementNumber exceeds that for the erased element it is decremented.
    void decrementRouteElementNumbersInRoute2MultiMap(int caller, int route_number, unsigned int erase_del_ement_number);
/// After a route has been erased from AllRoutesVector and its entries from Route2MultiMap, this function examines all the remaining entries in Route2MultiMap to see if their RouteNumbers exceed that for the erased route.  Where this is so the RouteNumber is decremented.
    void decrementRouteNumbersInRoute2MultiMap(int caller, int route_number);
/// Calls PrefDirMarker for all routes, with RouteCall set to identify a route call, and BuildingPrefDir false.
    // void markAllRoutes(int caller, TDisplay *Disp);
/// Erases the route element from Route2MultiMap and from the PrefDirVector.
    void removeRouteElement(int caller, int h_loc, int v_loc, int e_link);
/// Insert an entry in Route2MultiMap.  Called by TAllRoutes::AddRouteElement.
    void route2MultiMapInsert(int caller, int h_loc, int v_loc, int e_link_in, int route_number, unsigned int route_element_number);
/// Save railway route information to a session file or an error file
    void saveRoutes(int caller, std::ofstream &out_stream);
/// Set rearwards signals from the specified route starting position
    void setAllRearwardsSignals(int caller, int attribute, int route_number, int route_start_position);
/// Enter with signal at TrackVectorElement already set to red by the passing train.
/**Identify the route that the TrackVectorPosition is in,
carry out validity checks, then call SetAllRearwardsSignals to set signals in this route and all linked rearwards routes, unless find a train (a) in the current
route, in which case the signals behind it are set (and behind any other trains in the current route), but only within the current
route; or (b) in a linked rear route, in which case the function sets no further signals.*/
    void setTrailingSignalsOnAutoSigsRoute(int caller, int track_vector_position, int x_link_pos);
/// This is called by the InterfaceUnit at intervals based on entries in the ContinuationAutoSigVector in TrainController.
/**It sets signals on the AutoSigsRoute to correspond to a train having exited the route at a continuation, and passing further signals (outside the simulated railway).
Initially the last passed signal will be red, then at the first call it will change to yellow and earlier signals will change accordingly, then double
yellow, then green.  There are only 3 calls in all for any given route, and the AccessNumber changes from 0 to 1 to 2 for successive calls.*/
    void setTrailingSignalsOnContinuationRoute(int caller, int route_number, int access_number);
/// A new (empty apart from RouteID) TOneRoute is added to the AllRoutesVector.
/**The route is the last to be added, and will have a RouteNumber of AllRoutesSize() - 1.  Each element of the new route is added in
turn using AddRouteElement, which uses h_loc, v_loc, ELink and RouteNumber to provide the information necessary to insert it into both PrefDirVector and Route2MultiMap.*/
    void storeOneRoute(int caller, std::shared_ptr<OneRoute> route);
/// A new (empty apart from RouteID) TOneRoute is added to the AllRoutesVector after a session load. Very similar to StoreOneRoute but here the RouteID that is already in Route is used.
    void storeOneRouteAfterSessionLoad(int caller, std::shared_ptr<OneRoute> route);
/// Calls RouteImageMarker for each route in turn to display the route colours and direction arrows on the bitmap image (as on screen during operation) for an operating railway
    // void writeAllRoutesToImage(int caller, Graphics::TBitmap *Bitmap);
};

extern std::shared_ptr<Track> track;
extern std::shared_ptr<AllRoutes> all_routes;

}; // namespace RailOS::Track