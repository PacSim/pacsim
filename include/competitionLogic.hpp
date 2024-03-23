#include "types.hpp"
#include <iostream>
#include <string>

class CompetitionLogic
{
public:
    enum Discipline
    {
        AUTOCROSS,
        TRACKDRIVE,
        ACCELERATION,
        SKIDPAD
    };
    enum PENALTY_TYPE
    {
        DOO,
        OC,
        USS
    };

    struct Penalty
    {
        int lap;
        double time;
        PENALTY_TYPE reason;
        double penalty_time;
        Eigen::Vector3d position;
    };

    CompetitionLogic(Track& track, std::string disciplineStr);

    bool evaluateOffCourse(Track& track, double time, Eigen::Vector3d& position, Eigen::Vector3d& orientation);

    void evaluateConeHit(Track& track, double time, Eigen::Vector3d& position, Eigen::Vector3d& orientation);

    int timeKeepingStatus(
        Eigen::Vector3d lm1, Eigen::Vector3d lm2, Eigen::Vector3d& position, Eigen::Vector3d& orientation);

    void evaluateTimeKeepings(Track& track, Eigen::Vector3d& position, Eigen::Vector3d& orientation, double time);

    bool checkFinishConditionsMet(double time);

    bool checkUSS(Track track, double time, Eigen::Vector3d position);

    bool checkDNF(Track track, double time, Eigen::Vector3d position);

    bool performAllChecks(Track& track, double time, Eigen::Vector3d& position, Eigen::Vector3d& orientation);

    void setFinish(bool val);

    void fillReport(Report& report, double time);

private:
    bool pointInTriangle(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c, Eigen::Vector2d point);

    std::vector<bool> pointsInTrackConnected(Track& track, std::vector<Eigen::Vector2d> points);

    bool pointInPolygon(std::vector<Eigen::Vector2d> polyPoints, Eigen::Vector2d point);

    std::pair<bool, bool> rayIntersectLineSegment(
        Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d rayOrigin, Eigen::Vector2d rayDirection);

    double cross2d(Eigen::Vector2d a, Eigen::Vector2d b);

    std::vector<bool> pointsInTrackNotConnected(Track& track, std::vector<Eigen::Vector2d> points);

    bool carConePolyIntersect(std::vector<Eigen::Vector2d> carPoly, std::vector<Eigen::Vector2d> conePoly);

    double determinantLinePoint(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d c);

    bool inLineSegement(Eigen::Vector2d a, Eigen::Vector2d b, Eigen::Vector2d position);

    void evaluateTimeKeepingGateTrigger(Track track, double time, int index);

    std::string discipline2str(Discipline d);

    std::string penalty2str(PENALTY_TYPE p);

    std::vector<int> timeKeepingStatuses;

    std::vector<int> timeKeepingFirstTriggerStatuses;

    std::vector<std::vector<double>> triggerTimes;

    int lapCount;

    double lastTriggerTime;

    std::vector<double> lapTimes;
    std::vector<double> currentSectorTimes;
    std::vector<std::vector<double>> sectorTimes;
    Discipline discipline;
    double finishConditionsMetFirstTime;
    bool finishConditionsMet;
    bool alreadyOC;
    double Off_Course_Start;
    bool isDNF;
    std::string dnf_reason = "";
    std::vector<Penalty> penalties;
    bool ussTriggered;
    bool finishSignal;
    double timeout_first_lap;
    double timeout_total;
};