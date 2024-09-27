#include "types.hpp"

LandmarkType stringToLandmarkType(const std::string& in)
{
    LandmarkType ret = LandmarkType::UNKNOWN;
    if (in == "blue")
    {
        ret = LandmarkType::BLUE;
    }
    else if (in == "yellow")
    {
        ret = LandmarkType::YELLOW;
    }
    else if (in == "small-orange" || in == "orange")
    {
        ret = LandmarkType::ORANGE;
    }
    else if (in == "big-orange")
    {
        ret = LandmarkType::BIG_ORANGE;
    }
    else if (in == "timekeeping")
    {
        ret = LandmarkType::TIMEKEEPING;
    }
    else if (in == "invisible")
    {
        ret = LandmarkType::INVISIBLE;
    }
    return ret;
}

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
    for (auto& p : in.time_keeping_gates)
    {
        ret.list.push_back(p.first);
        ret.list.push_back(p.second);
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

Discipline stringToDiscipline(const std::string& disciplineStr)
{
    Discipline discipline = Discipline::AUTOCROSS;
    if (disciplineStr == "autocross")
    {
        discipline = Discipline::AUTOCROSS;
    }
    else if (disciplineStr == "trackdrive")
    {
        discipline = Discipline::TRACKDRIVE;
    }
    else if (disciplineStr == "acceleration")
    {
        discipline = Discipline::ACCELERATION;
    }
    else if (disciplineStr == "skidpad")
    {
        discipline = Discipline::SKIDPAD;
    }
    return discipline;
}