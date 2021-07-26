import airsim
import utm
import numpy as np

import geopy.distance
import yaml


class AirSimBackend(object):
    def __init__(self):
        self.objects = {}

    def request_data(self):
        self.objects = {}
        client = airsim.MultirotorClient()
        try:
            gps_vehicle = client.getGpsData()
            ue4_vehicle = client.simGetVehiclePose()
            object_names = [f"WindTurbine{i}" for i in range(50)]
            object_names.append("DroneLandingPlatform")
            for object_name in object_names:
                pose = client.simGetObjectPose(object_name)
                if not pose.containsNan():
                    result_dict = {}
                    lat, lng = self.convert_ue4_to_gps_coord(gps_vehicle, ue4_vehicle, pose)
                    result_dict["name"] = object_name
                    result_dict["gps"] = {"lat": lat,
                                          "lng": lng,
                                          "wind_direction": "??"}
                    result_dict["ue4_pose"] = {"x": round(pose.position.x_val, 4),
                                               "y": round(pose.position.y_val, 4),
                                               "yaw": round(pose.orientation.z_val, 4)}
                    result_dict["state"] = "UNKNOWN"  # Used to determine blade rotation so state 1,2, and unknown
                    result_dict["height"] = "100"  # Meters
                    result_dict["blade_radius"] = "40"  # Meters
                    result_dict["info_text"] = yaml.dump(result_dict)
                    self.objects[object_name] = result_dict

            client = None
            return self.objects
        except RuntimeError:
            client = None
            print("No connection established.")

    def convert_ue4_to_gps_coord(self, gps_anchor: airsim.GpsData, ue4_anchor: airsim.Pose, ue4_pose: airsim.Pose):
        """
        http://www.edwilliams.org/avform147.htm#LL
        :param gps_anchor:
        :param ue4_anchor:
        :param ue4_pose:
        :return:
        """
        # Anchor
        lat1 = gps_anchor.gnss.geo_point.latitude
        lon1 = gps_anchor.gnss.geo_point.longitude

        # Compute distance:
        relative_position = ue4_pose.position - ue4_anchor.position
        relative_position = np.array([relative_position.x_val, relative_position.y_val])
        relative_position_unit = relative_position / np.linalg.norm(relative_position)
        distance = np.linalg.norm(relative_position)

        # angle
        angle = np.arctan2(relative_position_unit[1], relative_position_unit[0]) - np.arctan2(1, 0)
        return self.gps_given_distance(lat1, lon1, distance, angle)

    def gps_given_distance(self, lat1, lon1, distance, angle):
        """
        :param lat1: degrees
        :param lon1: degrees
        :param distance: in meters
        :param angle: in radians
        :return:
        """
        # Define starting point.
        start = geopy.Point(lat1, lon1)

        # Define a general distance object, initialized with a distance of 1 km.
        d = geopy.distance.distance(kilometers=distance / 1000)

        # Use the `destination` method with a bearing of 0 degrees (which is north)
        # in order to go from point `start` 1 km to north.
        final = d.destination(point=start, bearing=np.rad2deg(angle))
        return final.latitude, final.longitude


if __name__ == '__main__':
    bg = AirSimBackend()
    while 1:
        distance = input("Enter distance:")
        angle = input("Enter angle:")
        test = bg.gps_given_distance(45, 45, int(distance), np.deg2rad(float(angle)))
        print(test)
