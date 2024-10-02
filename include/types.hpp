#ifndef PACSIMTYPES_HPP
#define PACSIMTYPES_HPP

#include "quaternion.hpp"
#include <Eigen/Core>
#include <vector>

enum LandmarkType
{
    BLUE = 0,
    YELLOW = 1,
    ORANGE = 2,
    BIG_ORANGE = 3,
    TIMEKEEPING = 4,
    INVISIBLE = 5,
    // convention: unknown is last element, that way we know the number of classes
    UNKNOWN = 6
};

LandmarkType stringToLandmarkType(const std::string& in);

// data structure to store everything for each Landmark (without color probabilities)
struct Landmark
{
    int id;
    Eigen::Vector3d position;
    Eigen::Matrix3d cov;
    bool beenHit = false;
    LandmarkType type = LandmarkType::UNKNOWN;
    double typeWeights[LandmarkType::UNKNOWN + 1] = { 0.0 };
    double detection_probability = 1.0;
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
    Eigen::Vector3d gnssOrigin;
    Eigen::Vector3d enuToTrackRotation;
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

struct GnssData
{
    double latitude;
    double longitude;
    double altitude;
    Eigen::Matrix3d position_covariance;

    double vel_east;
    double vel_north;
    double vel_up;
    Eigen::Matrix3d velocity_covariance;

    quaternion orientation;
    Eigen::Matrix3d orientation_covariance;

    enum FixStatus
    {
        NO_FIX = -1,
        FIX = 0
    };

    FixStatus fix_status;

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
    std::string dnf_reason = "";
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

enum Discipline
{
    AUTOCROSS,
    TRACKDRIVE,
    ACCELERATION,
    SKIDPAD
};

struct MainConfig
{
    double timeout_start;
    double timeout_acceleration;
    double timeout_autocross;
    double timeout_skidpad;
    double timeout_trackdrive_first;
    double timeout_trackdrive_total;
    bool oc_detect;
    bool doo_detect;
    bool uss_detect;
    bool finish_validate;
    Discipline discipline;
    std::string cog_frame_id_pipeline;
    bool broadcast_sensors_tf2;
    bool pre_transform_track;
};

Discipline stringToDiscipline(const std::string& disciplineStr);

#endif /* PACSIMTYPES_HPP */