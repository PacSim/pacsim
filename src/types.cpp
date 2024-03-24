#include "types.hpp"

LandmarkList trackToLMList(Track& in)
{
    LandmarkList ret;
    // TODO: no copy for lists
    std::vector<std::vector<Landmark>> lists { in.left_lane, in.right_lane, in.unknown };
    for (auto& list : lists)
    {
        for (Landmark lm : list)
        {
            ret.list.push_back(lm);
        }
    }
    return ret;
}

Track lmListToTrack(LandmarkList& in)
{
    Track ret;
    for (auto& lm : in.list)
    {
        ret.unknown.push_back(lm);
    }
    return ret;
}