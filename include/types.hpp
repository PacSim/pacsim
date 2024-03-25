#ifndef PACSIMTYPES_HPP
#define PACSIMTYPES_HPP

#include <Eigen/Core>
#include <vector>

// data structure to store everything for each Landmark (without color probabilities)
struct Landmark
{
    int id;
    Eigen::Vector3d position;
    Eigen::Matrix3d cov;
    bool beenHit = false;
    // ColorWeight weights;

    // double absolute_liveness;
    // int seen_ctr;
    // int life_time_ctr;

    // double first_seen;
    // double last_seen;
};

struct Map
{
    std::vector<Landmark> landmarks;
};

struct Track
{
    bool lanesFirstWithLastConnected = true;
    std::vector<Landmark> left_lane;
    std::vector<Landmark> right_lane;
    std::vector<Landmark> unknown;
    std::vector<std::pair<Landmark, Landmark>> time_keeping_gates;
};

struct LandmarkList
{
    std::vector<Landmark> list;
    std::vector<Eigen::Vector2d> fov_polygon;
    std::string source_type;

    std::string frame_id;
    double timestamp;
};

// TODO get rid of that
LandmarkList trackToLMList(Track& in);

Track lmListToTrack(LandmarkList& in);

// data structure to store everything for the Pose
struct Pose
{
    Eigen::Vector3d mean;
    Eigen::Matrix3d cov;
    double timestamp;

    std::string frame_id;
};
struct Wheels
{
    double FL;
    double FR;
    double RL;
    double RR;

    double timestamp;
};

struct ImuData
{
    Eigen::Vector3d acc;
    Eigen::Vector3d rot;

    Eigen::Matrix3d acc_cov;
    Eigen::Matrix3d rot_cov;

    double timestamp;
    std::string frame;
};

struct StampedScalar
{
    double data;

    double timestamp;
};

struct Report
{
    std::string creation_time = "";
    std::string track_name = "";
    std::string perception_config_file = "";
    std::string sensors_config_file = "";
    std::string vehicle_config_file = "";
    int seed = 0;

    std::string discipline = "";
    bool success;
    std::string reason = "";
    double final_time = 0.0;
    double final_time_raw = 0.0;
    double total_sim_time = 0.0;

    bool off_course_detect = true;
    bool cone_hit_detect = true;
    bool uss_detect = true;
    bool dnf_detect = true;
    bool finish_validate = true;
    double timeout_total = 300;
    double timeout_first_lap = 100;
    struct LapTime
    {
        double time;
        std::vector<double> sector_times;
    };
    std::vector<LapTime> lap_times;
    // std::vector<std::vector<double>> sector_times;

    struct Penalty
    {
        int lap;
        double penalty_time;
        double occurence_time;
        std::string reason;
        Eigen::Vector3d position;
    };
    std::vector<Penalty> penalties;
};

struct mainConfig
{
    double timeout_start;
    double timout_acceleration;
    double timeout_autocross;
    double timeout_skidpad;
    double timeout_total;
    bool oc_detect;
    bool doo_detect;
    bool uss_detect;
    bool finish_validate;
};

#endif /* PACSIMTYPES_HPP */