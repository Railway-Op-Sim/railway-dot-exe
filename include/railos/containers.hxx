#pragma once

#include <list>
#include <map>

#include "railos/positioning.hxx"

namespace RailOS::Containers {
    typedef std::list<int>NumList;
    ///< a list of valid train exit TrackVector positions for 'Fer' entries
    typedef NumList::iterator NumListIterator;

    ///these added for multiplayer
    typedef std::pair<int, int> HVShortPair;
    
    typedef std::multimap<HVShortPair, Positioning::ExitInfo> TimeToExitMultiMap;
    typedef std::pair<HVShortPair, Positioning::ExitInfo> TimeToExitMultiMapEntry;
};