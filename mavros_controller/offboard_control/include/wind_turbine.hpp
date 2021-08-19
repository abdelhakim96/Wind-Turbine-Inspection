#include <cstdlib>
#include <simple_json.hpp>
#include "common.h"

using namespace std;
using simple_json::JSON;

class gps_coord {
public:
    float lat;
    float lng;

    gps_coord() = default;;

    JSON to_json() const {
        JSON json_obj = {
                "lat", lat,
                "lng", lng
        };
        return json_obj;
    }

    gps_coord from_json(const JSON &j) {
        try {
            lat = j.at("lat").ToFloat();
            lng = j.at("lng").ToFloat();
        } catch (...) {
            ROS_INFO("Unable to convert gps_coord from json to class: %s", j.dump().c_str());
        }
        return *this;
    }
};

class UE4Pose {
public:
    float x, y, yaw;

    UE4Pose() = default;;

    JSON to_json() const {
        JSON json_obj = {
                "x", x,
                "y", y,
                "yaw", yaw
        };
        return json_obj;
    }

    UE4Pose from_json(const JSON &j) {
        try {
            x = j.at("x").ToFloat();
            y = j.at("y").ToFloat();
            yaw = j.at("yaw").ToFloat();
        } catch (...) {
            ROS_INFO("Unable to convert UE4Pose from json to class: %s", j.dump().c_str());
        }
        return *this;
    }
};

class WindTurbine {
public:
    std::string name;
    float height{};
    std::string state;
    std::string info_text;
    float blade_radius{};
    float canvas_pos[2] = {0, 0};
    gps_coord gpsCoord{};
    string windDirection;
    UE4Pose ue4_pose{};

    WindTurbine() = default;

    explicit WindTurbine(const JSON &j) {
        from_json(j);
    };

    JSON to_json() const {
        JSON json_obj = {
                "name", name,
                "height", height,
                "state", state,
                "info_text", info_text,
                "blade_radius", blade_radius,
                "canvas_pos", simple_json::Array(canvas_pos[0], canvas_pos[1]),
                "gpsCoord", gpsCoord.to_json(),
                "windDirection", windDirection,
                "ue4_pose", ue4_pose.to_json(),
        };
        return json_obj;
    }

    WindTurbine from_json(const JSON &j) {
        try {
            // Easy types
            name = j.at("name").ToString();
            height = j.at("height").ToFloat();
            state = j.at("state").ToString();
            info_text = j.at("info_text").ToString();
            blade_radius = j.at("blade_radius").ToFloat();
            windDirection = j.at("windDirection").ToString();

            // Harder types
            canvas_pos[0] = j.at("canvas_pos").at(0).ToFloat();
            canvas_pos[1] = j.at("canvas_pos").at(1).ToFloat();
            gpsCoord.from_json(j.at("gpsCoord"));
            ue4_pose.from_json("ue4_pose");
        } catch (...) {
            ROS_INFO("Unable to convert UE4Pose from json to class: %s", j.dump().c_str());
        }
        return *this;
    }

    string to_string() const {
        return to_json().dump();
    }

};