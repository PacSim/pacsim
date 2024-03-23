#include "competitionLogic.hpp"

CompetitionLogic::CompetitionLogic(Track& track, std::string disciplineStr)
{
    timeKeepingStatuses = std::vector<int>(track.time_keeping_gates.size(), 0);
    timeKeepingFirstTriggerStatuses = std::vector<int>(track.time_keeping_gates.size(), 0);
    triggerTimes = std::vector<std::vector<double>>(track.time_keeping_gates.size(), std::vector<double>());
    lapTimes = std::vector<double>();
    currentSectorTimes = std::vector<double>();
    sectorTimes = std::vector<std::vector<double>>();

    lapCount = 0;
    lastTriggerTime = 0;
    discipline = Discipline::AUTOCROSS;
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
    finishConditionsMetFirstTime = 0.0;
    finishConditionsMet = false;
    alreadyOC = false;
    Off_Course_Start = 0.0;
    isDNF = false;
    dnf_reason = "";
    penalties = std::vector<Penalty>();
    ussTriggered = false;
    finishSignal = false;
    timeout_first_lap = 300.0;
    timeout_total = 1000.0;
}

bool CompetitionLogic::pointInTriangle(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c, Eigen::Vector2d point)
{
    double as = ((b.y() - c.y()) * (point.x() - c.x()) + (c.x() - b.x()) * (point.y() - c.y()))
        / ((b.y() - c.y()) * (a.x() - c.x()) + (c.x() - b.x()) * (a.y() - c.y()));
    double bs = ((c.y() - a.y()) * (point.x() - c.x()) + (a.x() - c.x()) * (point.y() - c.y()))
        / ((b.y() - c.y()) * (a.x() - c.x()) + (c.x() - b.x()) * (a.y() - c.y()));
    double cs = 1 - as - bs;

    return ((as >= 0) && (as <= 1) && (bs >= 0) && (bs <= 1) && (cs >= 0) && (cs <= 1));
}

std::vector<bool> CompetitionLogic::pointsInTrackConnected(Track& track, std::vector<Eigen::Vector2d> points)
{
    std::vector<size_t> leftToRight;
    leftToRight.reserve(track.left_lane.size());
    std::vector<size_t> rightToLeft;
    rightToLeft.reserve(track.right_lane.size());
    Eigen::Vector2d referencePoint = points.at(0);

    size_t currentLeftIndex = 0;
    for (size_t i = 0; i < track.left_lane.size(); ++i)
    {
        auto left_cone = track.left_lane.at(i);
        // search in limited window
        size_t argmin_local = 0;
        double min_dist_local = std::numeric_limits<double>::max();
        for (size_t j = 0; j < 5; ++j)
        {
            auto right_cone = track.right_lane.at((j + currentLeftIndex) % track.right_lane.size());
            double dist = (left_cone.position - right_cone.position).norm();
            if (dist < min_dist_local)
            {
                argmin_local = j;
                min_dist_local = dist;
            }
        }
        currentLeftIndex += argmin_local;
        leftToRight.push_back(currentLeftIndex);
    }

    size_t currentRightIndex = 0;
    for (size_t i = 0; i < track.right_lane.size(); ++i)
    {
        auto right_cone = track.right_lane.at(i);
        // search in limited window
        size_t argmin_local = 0;
        double min_dist_local = std::numeric_limits<double>::max();
        for (size_t j = 0; j < 5; ++j)
        {
            auto left_cone = track.left_lane.at((j + currentRightIndex) % track.left_lane.size());
            double dist = (left_cone.position - right_cone.position).norm();
            if (dist < min_dist_local)
            {
                argmin_local = j;
                min_dist_local = dist;
            }
        }
        currentRightIndex += argmin_local;
        rightToLeft.push_back(currentRightIndex);
    }

    // step 2 find closest cone to car
    double bestDist = std::numeric_limits<double>::max();
    size_t indexLeft;
    size_t indexRight;
    for (size_t i = 0; i < track.left_lane.size(); ++i)
    {
        double dist = (track.left_lane[i].position.head(2) - referencePoint).norm();
        if (dist < bestDist)
        {
            indexLeft = i;
            indexRight = leftToRight.at(i);
            bestDist = dist;
        }
    }
    for (size_t i = 0; i < track.right_lane.size(); ++i)
    {
        double dist = (track.right_lane[i].position.head(2) - referencePoint).norm();
        if (dist < bestDist)
        {
            indexLeft = rightToLeft.at(i);
            indexRight = i;
            bestDist = dist;
        }
    }
    // build triangles from left lane
    std::vector<std::vector<Eigen::Vector2d>> triPoints;
    for (int i = -7; i < 7; ++i)
    {
        Eigen::Vector2d left1
            = track.left_lane.at((indexLeft + i - 1 + track.left_lane.size()) % track.left_lane.size())
                  .position.head(2);
        Eigen::Vector2d left2
            = track.left_lane.at((indexLeft + i + track.left_lane.size()) % track.left_lane.size()).position.head(2);
        Eigen::Vector2d right1
            = track.right_lane
                  .at((leftToRight.at((indexLeft + i - 1 + track.left_lane.size()) % track.left_lane.size())
                          + track.right_lane.size())
                      % track.right_lane.size())
                  .position.head(2);
        Eigen::Vector2d right2
            = track.right_lane
                  .at((leftToRight.at((indexLeft + i + track.left_lane.size()) % track.left_lane.size())
                          + track.right_lane.size())
                      % track.right_lane.size())
                  .position.head(2);
        std::vector<Eigen::Vector2d> tmpVector { left1, left2, right1 };
        std::vector<Eigen::Vector2d> tmpVector2 { left1, left2, right2 };
        triPoints.push_back(tmpVector);
        triPoints.push_back(tmpVector2);
    }
    // build triangles from right lane
    for (int i = -7; i < 7; ++i)
    {
        Eigen::Vector2d right1
            = track.right_lane.at((indexLeft + i - 1 + track.right_lane.size()) % track.right_lane.size())
                  .position.head(2);
        Eigen::Vector2d right2
            = track.right_lane.at((indexLeft + i + track.right_lane.size()) % track.right_lane.size()).position.head(2);
        Eigen::Vector2d left1
            = track.left_lane
                  .at((rightToLeft.at((indexRight + i - 1 + track.right_lane.size()) % track.right_lane.size())
                          + track.left_lane.size())
                      % track.left_lane.size())
                  .position.head(2);
        Eigen::Vector2d left2
            = track.left_lane
                  .at((rightToLeft.at((indexRight + i + track.right_lane.size()) % track.right_lane.size())
                          + track.left_lane.size())
                      % track.left_lane.size())
                  .position.head(2);
        std::vector<Eigen::Vector2d> tmpVector { right1, right2, left1 };
        std::vector<Eigen::Vector2d> tmpVector2 { right1, right2, left2 };
        triPoints.push_back(tmpVector);
        triPoints.push_back(tmpVector2);
    }
    // check all triangels
    std::vector<bool> ret;
    for (Eigen::Vector2d& point : points)
    {
        bool inTrack = false;
        for (int i = 0; i < triPoints.size(); ++i)
        {
            inTrack = inTrack
                || pointInTriangle(triPoints.at(i).at(0), triPoints.at(i).at(1), triPoints.at(i).at(2), point);
        }
        ret.push_back(inTrack);
    }
    return ret;
}

double CompetitionLogic::cross2d(Eigen::Vector2d a, Eigen::Vector2d b) { return (a.x() * b.y() - a.y() * b.x()); }

std::pair<bool, bool> CompetitionLogic::rayIntersectLineSegment(
    Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d rayOrigin, Eigen::Vector2d rayDirection)
{
    // thank you gareth rees
    // https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect/565282#565282
    Eigen::Vector2d segDirection = b - a;
    double RayDxSegD = cross2d(rayDirection, segDirection);
    double aRayOxRayD = cross2d(a - rayOrigin, rayDirection);
    // colinear
    if (RayDxSegD == 0 && aRayOxRayD == 0)
    {
        return std::make_pair(false, true);
    }
    // parallel
    else if (RayDxSegD == 0 && aRayOxRayD != 0)
    {
        return std::make_pair(false, false);
    }
    // may intersect
    else if (RayDxSegD != 0)
    {
        double u = aRayOxRayD / RayDxSegD;
        double t = cross2d(a - rayOrigin, segDirection) / RayDxSegD;
        if (t > 0 && (u >= 0) && (u <= 1))
        {
            return std::make_pair(true, false);
        }
        else
        {
            return std::make_pair(false, false);
        }
        double x = 1;
    }
    // not parallel but no intersect
    else
    {
        return std::make_pair(false, false);
    }
}

bool CompetitionLogic::pointInPolygon(std::vector<Eigen::Vector2d> polyPoints, Eigen::Vector2d point)
{
    // use ray-casting approach https://en.wikipedia.org/wiki/Point_in_polygon
    // uneven amount of intersections -> in polygon, else not in polygon
    bool ret = false;
    int intersectCount = 0;
    double rayAngle = 0.01;
    size_t iters = 0;
    bool tryRays = true;
    while (tryRays)
    {
        // if we tried too often, screw it and just don't trigger a false positive.
        if (iters > 30)
        {
            return true;
        }
        iters += 1;
        Eigen::Vector2d rayDirection(std::cos(rayAngle), std::sin(rayAngle));
        for (int i = 1; i < polyPoints.size(); ++i)
        {
            std::pair<bool, bool> intersectRes
                = rayIntersectLineSegment(polyPoints[i - 1], polyPoints[i], point, Eigen::Vector2d(1, 0));
            // if ray is colinear with polygon edge, vary ray direction slightly
            if (intersectRes.second)
            {
                intersectCount = 0;
                rayAngle += 0.01;
                break;
            }
            if (intersectRes.first)
            {
                intersectCount += 1;
            }
        }
        tryRays = false;
    }
    ret = (intersectCount % 2) == 1;
    return ret;
}

std::vector<bool> CompetitionLogic::pointsInTrackNotConnected(Track& track, std::vector<Eigen::Vector2d> points)
{
    std::vector<Eigen::Vector2d> polyPoints;
    for (Landmark& lm : track.left_lane)
    {
        polyPoints.push_back(lm.position.head(2));
    }
    for (int i = (track.right_lane.size() - 1); i >= 0; --i)
    {
        polyPoints.push_back(track.right_lane[i].position.head(2));
    }
    polyPoints.push_back(track.left_lane[0].position.head(2));

    Eigen::Vector2d point(0, 0);
    std::vector<bool> ret;
    for (auto& point : points)
    {
        ret.push_back(pointInPolygon(polyPoints, point));
    }
    return ret;
}

bool CompetitionLogic::evaluateOffCourse(
    Track& track, double time, Eigen::Vector3d& position, Eigen::Vector3d& orientation)
{
    double lf = 0.75;
    double lr = 0.75;
    double halfwidth = 0.6;
    Eigen::Vector2d flLocal(lf, halfwidth);
    Eigen::Vector2d frLocal(lf, -halfwidth);
    Eigen::Vector2d rlLocal(-lr, halfwidth);
    Eigen::Vector2d rrLocal(-lr, -halfwidth);
    Eigen::Matrix2d rotM;
    rotM << std::cos(orientation.z()), -std::sin(orientation.z()), std::sin(orientation.z()), std::cos(orientation.z());
    Eigen::Vector2d fl = position.head(2) + rotM * flLocal;
    Eigen::Vector2d fr = position.head(2) + rotM * frLocal;
    Eigen::Vector2d rl = position.head(2) + rotM * rlLocal;
    Eigen::Vector2d rr = position.head(2) + rotM * rrLocal;
    std::vector<Eigen::Vector2d> pointsToBeChecked { fl, fr, rl, rr };
    std::vector<bool> inTrack;
    if (track.lanesFirstWithLastConnected)
    {
        inTrack = pointsInTrackConnected(track, pointsToBeChecked);
    }
    else
    {
        inTrack = pointsInTrackNotConnected(track, pointsToBeChecked);
    }
    bool flOk = inTrack.at(0);
    bool frOk = inTrack.at(1);
    bool rlOk = inTrack.at(2);
    bool rrOk = inTrack.at(3);

    bool inLane = flOk || frOk || rlOk || rrOk;
    if (!inLane)
    {
        if ((!alreadyOC) && (discipline == Discipline::AUTOCROSS || discipline == Discipline::TRACKDRIVE))
        {
            Penalty p;
            p.lap = lapTimes.size();
            p.penalty_time = time;
            p.reason = PENALTY_TYPE::OC;
            p.time = 10.0;
            penalties.push_back(p);
        }
        Off_Course_Start = time;
    }
    alreadyOC = !inLane;
    return inLane;
}

bool CompetitionLogic::carConePolyIntersect(std::vector<Eigen::Vector2d> carPoly, std::vector<Eigen::Vector2d> conePoly)
{
    // assumptions: car shape is a pretty much a rectangle and much bigger than cone, so point in poly is enough of a
    // check
    bool result = false;
    // dont need to check last point because it's equal to first one
    for (int i = 0; i < (conePoly.size() - 1); ++i)
    {
        result = result || pointInPolygon(carPoly, conePoly.at(i));
    }
    for (int i = 0; i < (carPoly.size() - 1); ++i)
    {
        result = result || pointInPolygon(conePoly, carPoly.at(i));
    }
    return result;
}

void CompetitionLogic::evaluateConeHit(
    Track& track, double time, Eigen::Vector3d& position, Eigen::Vector3d& orientation)
{
    double lf = 0.75 + 0.9;
    double lr = 0.75 + 0.3;
    double halfwidth = 0.6;
    Eigen::Vector2d flLocal(lf, halfwidth);
    Eigen::Vector2d frLocal(lf, -halfwidth);
    Eigen::Vector2d rlLocal(-lr, halfwidth);
    Eigen::Vector2d rrLocal(-lr, -halfwidth);
    Eigen::Matrix2d rotM;
    rotM << std::cos(orientation.z()), -std::sin(orientation.z()), std::sin(orientation.z()), std::cos(orientation.z());
    Eigen::Vector2d fl = position.head(2) + rotM * flLocal;
    Eigen::Vector2d fr = position.head(2) + rotM * frLocal;
    Eigen::Vector2d rl = position.head(2) + rotM * rlLocal;
    Eigen::Vector2d rr = position.head(2) + rotM * rrLocal;
    std::vector<Eigen::Vector2d> pointsToBeChecked { fl, fr, rr, rl, fl };

    double coneCheckDist = 5.0;
    double coneWidth = 0.228;
    std::vector<Landmark*> closeCones;
    double closeThresh = 5.0;
    for (int i = 0; i < track.left_lane.size(); ++i)
    {
        if ((position.head(2) - track.left_lane.at(i).position.head(2)).norm() <= closeThresh)
        {
            closeCones.push_back(&track.left_lane.at(i));
        }
    }
    for (int i = 0; i < track.right_lane.size(); ++i)
    {
        if ((position.head(2) - track.right_lane.at(i).position.head(2)).norm() <= closeThresh)
        {
            closeCones.push_back(&track.right_lane.at(i));
        }
    }
    for (int i = 0; i < track.unknown.size(); ++i)
    {
        if ((position.head(2) - track.unknown.at(i).position.head(2)).norm() <= closeThresh)
        {
            closeCones.push_back(&track.unknown.at(i));
        }
    }

    for (int i = 0; i < closeCones.size(); ++i)
    {
        Eigen::Vector2d conePos = closeCones.at(i)->position.head(2);
        Eigen::Vector2d p1 = conePos + Eigen::Vector2d(coneWidth * 0.5, coneWidth * 0.5);
        Eigen::Vector2d p2 = conePos + Eigen::Vector2d(coneWidth * 0.5, -coneWidth * 0.5);
        Eigen::Vector2d p3 = conePos + Eigen::Vector2d(-coneWidth * 0.5, -coneWidth * 0.5);
        Eigen::Vector2d p4 = conePos + Eigen::Vector2d(-coneWidth * 0.5, coneWidth * 0.5);
        std::vector<Eigen::Vector2d> conePoints { p1, p2, p3, p4, p1 };
        bool hitStatus = carConePolyIntersect(pointsToBeChecked, conePoints);
        if (!closeCones.at(i)->beenHit && hitStatus)
        {
            Penalty p;
            p.lap = lapTimes.size();
            p.penalty_time = time;
            p.reason = PENALTY_TYPE::DOO;
            p.time = 2.0;
            if (discipline == Discipline::SKIDPAD)
            {
                p.time = 0.2;
            }
            penalties.push_back(p);
        }
        closeCones.at(i)->beenHit = closeCones.at(i)->beenHit || hitStatus;
    }

    return;
}

double CompetitionLogic::determinantLinePoint(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c)
{
    double ret = (b.x() - a.x()) * (c.y() - a.y()) - (c.x() - a.x()) * (b.y() - a.y());
    return ret;
}

bool CompetitionLogic::inLineSegement(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d position)
{
    Eigen::Vector2d slope = b - a;
    Eigen::Vector2d slopeOrtho = Eigen::Vector2d(-slope.y(), slope.x()).normalized();
    Eigen::Vector2d orthoBack1 = a - slopeOrtho;
    Eigen::Vector2d orthoBack2 = a + slopeOrtho;
    Eigen::Vector2d orthoFront1 = b - slopeOrtho;
    Eigen::Vector2d orthoFront2 = b + slopeOrtho;
    bool backOk = determinantLinePoint(orthoBack1, orthoBack2, position) <= 0;
    bool frontOk = determinantLinePoint(orthoFront1, orthoFront2, position) >= 0;
    bool ret = backOk && frontOk;
    return ret;
}

int CompetitionLogic::timeKeepingStatus(
    Eigen::Vector3d lm1, Eigen::Vector3d lm2, Eigen::Vector3d& position, Eigen::Vector3d& orientation)
{
    Eigen::Vector2d transponderPosition = position.head(2);
    int ret = 0;
    if (inLineSegement(lm1.head(2), lm2.head(2), transponderPosition))
    {
        double valLine = determinantLinePoint(lm1.head(2), lm2.head(2), transponderPosition);
        ret = 1 + static_cast<int>(valLine <= 0);
    }
    return ret;
}

void CompetitionLogic::evaluateTimeKeepingGateTrigger(Track track, double time, int index)
{
    int i = index;
    // start/finish
    if (i == 0)
    {
        if (triggerTimes[i].size() >= 1)
        {
            double timeDiff = time - triggerTimes[0][triggerTimes[0].size() - 1];
            lapTimes.push_back(timeDiff);
            sectorTimes.push_back(currentSectorTimes);
            currentSectorTimes.clear();
        }
    }
    else if (i == 1 && !track.lanesFirstWithLastConnected && discipline != Discipline::SKIDPAD)
    {
        if (triggerTimes[0].size() >= 1)
        {
            double timeDiff = time - triggerTimes[0][triggerTimes[0].size() - 1];
            lapTimes.push_back(timeDiff);
            sectorTimes.push_back(currentSectorTimes);
            currentSectorTimes.clear();
        }
    }
    // sector time
    else
    {
        double sectorTime = time - lastTriggerTime;
        currentSectorTimes.push_back(time);
    }
    triggerTimes[i].push_back(time);
    lastTriggerTime = time;
}

void CompetitionLogic::evaluateTimeKeepings(
    Track& track, Eigen::Vector3d& position, Eigen::Vector3d& orientation, double time)
{
    for (int i = 0; i < track.time_keeping_gates.size(); ++i)
    {
        int res = this->timeKeepingStatus(track.time_keeping_gates[i].first.position,
            track.time_keeping_gates[i].second.position, position, orientation);
        if (res != 0)
        {
            // know which side we first get close to gate, that's "behind"
            if (timeKeepingStatuses[i] == 0)
            {
                timeKeepingFirstTriggerStatuses[i] = res;
            }
            // we just got in front of the gate, trigger lap
            if ((timeKeepingStatuses[i] != res) && (res != timeKeepingFirstTriggerStatuses[i]))
            {
                evaluateTimeKeepingGateTrigger(track, time, i);
            }
            timeKeepingStatuses[i] = res;
        }
    }
}

bool CompetitionLogic::checkFinishConditionsMet(double time)
{
    bool ret = false;
    if (discipline == Discipline::SKIDPAD)
    {
        ret = (lapTimes.size() == 4 && triggerTimes[1].size() >= 1);
    }
    if (discipline == Discipline::ACCELERATION)
    {
        ret = (triggerTimes[1].size() >= 1);
    }
    if (discipline == Discipline::AUTOCROSS)
    {
        ret = lapTimes.size() >= 1;
    }
    if (discipline == Discipline::TRACKDRIVE)
    {
        ret = lapTimes.size() >= 10;
    }
    if (ret && !finishConditionsMet)
    {
        finishConditionsMet = ret;
        finishConditionsMetFirstTime = time;
    }
    return ret;
}

double distanceToLineSegment(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d point)
{
    // https://stackoverflow.com/a/6853926
    double x1 = a.x();
    double x2 = b.x();
    double y1 = a.y();
    double y2 = b.y();
    double x = point.x();
    double y = point.y();
    double A = x - x1;
    double B = y - y1;
    double C = x2 - x1;
    double D = y2 - y1;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = -1;
    if (len_sq != 0) // in case of 0 length line
        param = dot / len_sq;

    double xx, yy;

    if (param < 0)
    {
        xx = x1;
        yy = y1;
    }
    else if (param > 1)
    {
        xx = x2;
        yy = y2;
    }
    else
    {
        xx = x1 + param * C;
        yy = y1 + param * D;
    }

    double dx = x - xx;
    double dy = y - yy;
    double ret = std::sqrt(dx * dx + dy * dy);
    return ret;
}

bool CompetitionLogic::checkUSS(Track track, double time, Eigen::Vector3d position)
{
    bool ret = false;
    if (finishConditionsMet)
    {
        if (discipline == Discipline::AUTOCROSS || discipline == Discipline::TRACKDRIVE)
        {
            ret = ret
                || (distanceToLineSegment(track.time_keeping_gates[0].first.position.head(2),
                        track.time_keeping_gates[0].second.position.head(2), position.head(2))
                    >= 30.0);
        }
        if (!ussTriggered && discipline == Discipline::TRACKDRIVE)
        {
            Penalty p;
            p.lap = lapTimes.size();
            p.penalty_time = time;
            p.reason = PENALTY_TYPE::USS;
            p.time = 10.0;
            penalties.push_back(p);
        }
        // a bit simplified because a few seconds can pass between passing line and standing in finish area
        ret = ret || ((time - finishConditionsMetFirstTime) >= (30 + 3));
    }

    return ret;
}

bool CompetitionLogic::checkDNF(Track track, double time, Eigen::Vector3d position)
{
    bool ret = false;
    if (discipline != Discipline::TRACKDRIVE)
    {
        if (checkUSS(track, time, position))
        {
            ret = true;
            dnf_reason = "USS";
        }
    }
    if (alreadyOC)
    {
        bool any = false;
        any = any || (discipline == Discipline::ACCELERATION);
        any = any || (discipline == Discipline::SKIDPAD);
        // excessive off-course check
        // my definition: too long or too far out
        // TODO also check too far our case
        any = any || ((time - Off_Course_Start) >= 8);
        if (any)
        {
            ret = true;
            dnf_reason = "OC";
        }
    }
    // TODO timeout case
    // TODO skidpad wrong circle
    isDNF = isDNF || ret;
    return isDNF;
}

bool CompetitionLogic::performAllChecks(
    Track& track, double time, Eigen::Vector3d& position, Eigen::Vector3d& orientation)
{
    // return true when simulation should stop
    evaluateOffCourse(track, time, position, orientation);
    evaluateTimeKeepings(track, position, orientation, time);
    checkFinishConditionsMet(time);
    checkUSS(track, time, position);
    evaluateConeHit(track, time, position, orientation);
    bool ret = checkDNF(track, time, position);
    ret = ret || finishSignal;
    return ret;
}

void CompetitionLogic::setFinish(bool val)
{
    finishSignal = val;
    return;
}

std::string CompetitionLogic::discipline2str(Discipline d)
{
    std::string ret = "unknown";
    if (d == Discipline::AUTOCROSS)
    {
        ret = "autocross";
    }
    else if (d == Discipline::TRACKDRIVE)
    {
        ret = "trackdrive";
    }
    else if (d == Discipline::ACCELERATION)
    {
        ret = "acceleration";
    }
    else if (d == Discipline::SKIDPAD)
    {
        ret = "skidpad";
    }
    return ret;
}

std::string CompetitionLogic::penalty2str(PENALTY_TYPE p)
{
    std::string ret = "unknown";
    if (p == PENALTY_TYPE::DOO)
    {
        ret = "doo";
    }
    else if (p == PENALTY_TYPE::OC)
    {
        ret = "oc";
    }
    else if (p == PENALTY_TYPE::USS)
    {
        ret = "uss";
    }
    return ret;
}

void CompetitionLogic::fillReport(Report& report, double time)
{
    report.discipline = discipline2str(discipline);
    report.success = (!isDNF) && (finishSignal && checkFinishConditionsMet(time));
    report.total_sim_time = time;
    report.final_time_raw = 0;
    report.timeout_first_lap = timeout_first_lap;
    report.timeout_total = timeout_total;
    report.off_course_detect = true;
    report.cone_hit_detect = true;
    report.uss_detect = true;

    for (auto t : lapTimes)
    {
        report.final_time_raw += t;
    }
    report.final_time = report.final_time_raw;
    for (int i = 0; i < lapTimes.size(); ++i)
    {
        Report::LapTime time;
        time.time = lapTimes[i];
        std::vector<double> sectors;
        for (int j = 0; j < sectorTimes[i].size(); ++j)
        {
            sectors.push_back(sectorTimes[i][j]);
        }
        time.sector_times = sectors;
        report.lap_times.push_back(time);
    }
    for (int i = 0; i < penalties.size(); ++i)
    {
        Report::Penalty p;
        p.lap = penalties[i].lap;
        p.penalty_time = penalties[i].penalty_time;
        report.final_time = p.penalty_time;
        p.occurence_time = penalties[i].time;
        p.position = penalties[i].position;
        p.reason = penalty2str(penalties[i].reason);
        report.penalties.push_back(p);
    }
}