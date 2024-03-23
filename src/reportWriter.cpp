#include "reportWriter.hpp"

bool reportToFile(Report report, std::string dir)
{
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "report";
    out << YAML::BeginMap;
    out << YAML::Key << "version";
    out << YAML::Value << "0.9";
    out << YAML::Key << "creation_time";
    time_t rawtime;

    struct tm* timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, 80, "%F %T", timeinfo);
    out << YAML::Value << buffer;
    out << YAML::Key << "seed";
    out << YAML::Value << report.seed;
    out << YAML::Key << "track_file";
    out << YAML::Value << report.track_name;
    out << YAML::Key << "perception_config_file";
    out << YAML::Value << report.perception_config_file;
    out << YAML::Key << "sensors_config_file";
    out << YAML::Value << report.sensors_config_file;
    out << YAML::Key << "vehicle_config_file";
    out << YAML::Value << report.vehicle_config_file;

    out << YAML::Key << "status";
    out << YAML::BeginMap;
    out << YAML::Key << "discipline";
    out << YAML::Value << report.discipline;
    out << YAML::Key << "success";
    out << YAML::Value << report.success;
    out << YAML::Key << "reason";
    out << YAML::Value << report.reason;
    out << YAML::Key << "final_time";
    out << YAML::Value << report.final_time;
    out << YAML::Key << "final_time_raw";
    out << YAML::Value << report.final_time_raw;
    out << YAML::Key << "total_sim_time";
    out << YAML::Value << report.total_sim_time;
    out << YAML::EndMap;

    out << YAML::Key << "settings";
    out << YAML::BeginMap;
    out << YAML::Key << "timeout-first-lap";
    out << YAML::Value << report.timeout_first_lap;
    out << YAML::Key << "timeout-total";
    out << YAML::Value << report.timeout_total;
    out << YAML::Key << "oc_detect";
    out << YAML::Value << report.off_course_detect;
    out << YAML::Key << "doo_detect";
    out << YAML::Value << report.cone_hit_detect;
    out << YAML::Key << "uss_detect";
    out << YAML::Value << report.uss_detect;
    out << YAML::Key << "dnf_detect";
    out << YAML::Value << report.dnf_detect;
    out << YAML::Key << "finish_validate";
    out << YAML::Value << report.finish_validate;
    out << YAML::EndMap;

    out << YAML::Key << "laps";
    out << YAML::BeginSeq;
    for (int i = 0; i < report.lap_times.size(); ++i)
    {
        out << YAML::BeginMap;
        out << YAML::Key << "lap";
        out << YAML::BeginMap;
        out << YAML::Key << "time";
        out << YAML::Value << report.lap_times[i].time;
        out << YAML::Key << "sectors";
        out << YAML::Flow << YAML::BeginSeq;
        for (int j = 0; j < report.lap_times[i].sector_times.size(); ++j)
        {
            out << report.lap_times[i].sector_times[j];
        }
        out << YAML::EndSeq;
        out << YAML::EndMap;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    out << YAML::Key << "penalties";
    out << YAML::BeginSeq;
    for (int i = 0; i < report.penalties.size(); ++i)
    {
        out << YAML::BeginMap;
        out << YAML::Key << "penalty";
        out << YAML::BeginMap;
        out << YAML::Key << "lap";
        out << YAML::Value << report.penalties[i].lap;
        out << YAML::Key << "time";
        out << YAML::Value << report.penalties[i].penalty_time;
        out << YAML::Key << "occurence_time";
        out << YAML::Value << report.penalties[i].occurence_time;
        out << YAML::Key << "reason";
        out << YAML::Value << report.penalties[i].reason;
        out << YAML::Key << "car_position";
        out << YAML::Flow << YAML::BeginSeq;
        out << report.penalties[i].position.x() << report.penalties[i].position.y() << report.penalties[i].position.z();
        out << YAML::EndSeq;
        out << YAML::EndMap;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    out << YAML::EndMap;
    out << YAML::EndMap;
    strftime(buffer, 80, "%F-%H-%M-%S", timeinfo);
    std::string path = dir + "/report-" + std::string(buffer) + ".yaml";
    std::ofstream fout(path);
    fout << out.c_str();

    return true;
}