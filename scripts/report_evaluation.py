#!/usr/bin/python3
import yaml
import sys
import os


def main(path):
    with open(path) as f:
        report = yaml.full_load(f)
        success = evaluate(report)
        output(report, success, os.path.splitext(path)[0])
        return success


def evaluate(report: dict):
    if "report" not in report.keys():
        raise RuntimeError(
            "The specified report does not follow the standard formatting")
    report_contents = report["report"]
    if "status" not in report_contents.keys():
        raise RuntimeError(
            "The specified report does not have a status associated")
    status = report_contents["status"]
    if "success" not in status:
        raise RuntimeError(
            "The specified report does not provide success information")
    return bool(status["success"])


def output(report, success, file_path):
    report = report["report"]
    f = open(file_path + ".txt", "w+")

    if "discipline" in report["status"]:
        f.write(f"Discipline: {report['status']['discipline']}\n")
    if "track_file" in report:
        f.write(f"Track: {report['track_file']}\n")
    f.write(f"Success: {':white_check_mark:' if success else ':x:'}\n")
    if "final_time_raw" in report['status']:
        f.write(f"Driven Time: {report['status']['final_time_raw']}\n")

    penalty_time = 0
    for penalty in report["penalties"]:
        if penalty["penalty"]["time"]:
            penalty_time += penalty["penalty"]["time"]

    f.write(f"Penalty time: {penalty_time}")


if __name__ == "__main__":
    try:
        path = str(sys.argv[1])
        main(path)
    except IndexError:
        raise IndexError(
            "Path to the report file not specified, expected call: report_evaluation.py {path}")
