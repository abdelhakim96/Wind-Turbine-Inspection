import base64
import threading

import cv2
import numpy as np
from flask import Flask, url_for, render_template, Response, request, redirect, Blueprint

from backend.airsim_backend import AirSimBackend
from backend.windprofile import WindFarm

inspection_page_bp = Blueprint("inspection_page_bp",
                               __name__,
                               static_folder="../../static",
                               template_folder="../../templates")

airsim_backend = AirSimBackend(debug_mode=False)
mission_info_dict = {"target": ""}
wind_direction = 0
wind_speed = 0
windfarm = WindFarm(wind_speed=0.1, wind_direction=0)
wind_sim = False

# lock to control access to variable
dataLock = threading.Lock()
# thread handler
yourThread = threading.Thread()
POOL_TIME = 0.5  # Seconds


@inspection_page_bp.route('/')
def inspection_page():
    global wind_sim
    wind_sim = False
    windfarm.wts_list = []
    canvas_size = (800, 800)
    # AirSim, check for windmill poses
    data = airsim_backend.request_data()

    drawing_data = []
    for k, v in data.items():
        drawing_data.append([v["gps"]["lat"], v["gps"]["lng"]])
    drawing_data = np.array(drawing_data)
    centroid = np.mean(drawing_data, axis=0)
    drawing_data_center = drawing_data[:] - centroid

    # Compute drawing position
    buffer = 40  # To make them not spawn on the border
    circle_size = 25  # To remove the size of the dot from borders
    w, h = canvas_size  # To know how big the canvas is and how much we can scale each point
    x_axis = drawing_data_center[:, 0]
    y_axis = drawing_data_center[:, 1]
    scale_x = (w - (circle_size + buffer)) / (np.max(x_axis) - np.min(x_axis))
    scale_y = (h - (circle_size + buffer)) / (np.max(y_axis) - np.min(y_axis))
    scale = min(scale_x, scale_y)
    drawing_data_center = drawing_data_center * scale - np.array([np.min(x_axis) * scale, np.min(y_axis) * scale])
    if scale_x > scale_y:
        center = (np.max(x_axis * scale) - np.min(x_axis * scale)) / 2
        offset = w / 2 - center
        drawing_data_center += [offset, 0]
    else:
        center = (np.max(y_axis * scale) - np.min(y_axis * scale)) / 2
        offset = h / 2 - center
        drawing_data_center += [0, offset]

    for i, object_name in enumerate(data.keys()):
        x, y = drawing_data_center[i]
        data[object_name]["drawing_data"] = {"canvas_pos": [x + buffer / 2, y + buffer / 2]}

    # Compute default windflow
    for k, v in data.items():
        if "Turbine" in k:
            windfarm.add_wind_turbine(name=v["name"],
                                      type_name="SimTurbine",
                                      diameter=float(v['blade_radius']) * 2,
                                      hub_height=float(v['height']),
                                      xy_pos=[v['ue4_pose']['y'], v['ue4_pose']['x']])
    flow_map, flow_data = windfarm.generate_flow_map(map_size=canvas_size[0], gen_flow_box=False)
    retval, buffer = cv2.imencode('.png', flow_map)
    png_as_text = base64.b64encode(buffer)
    page_info = {
        'windturbine_data': data,
        'flow_map': png_as_text.decode('utf-8')
    }
    return render_template("inspection.html", page_info=page_info, canvas_size=canvas_size)


@inspection_page_bp.route("/mission_info", methods=["POST", "GET"])
def mission_info():
    if request.method == "POST":
        pass
    # Do POST stuff
    elif request.method == "GET":
        if mission_info_dict["target"] != "":
            responds = {"target_name": mission_info_dict["target"],
                        "map_info": airsim_backend.objects}
            return responds
        else:
            return {"responds": "NO mission found"}


@inspection_page_bp.route("/update_backend/<object_name>", methods=["POST"])
def update_backend(object_name):
    wind_turbines = airsim_backend.objects
    if object_name in wind_turbines:
        wind_turbines[object_name]["state"] = request.form["state"]
        wind_turbines[object_name]["height"] = request.form["turbine_height"]
        wind_turbines[object_name]["blade_radius"] = request.form["blade_radius"]
        mission_info_dict["target"] = object_name
    return ("updated", 204)


@inspection_page_bp.route("/set_wind_direction", methods=["POST"])
def set_wind_direction():
    global wind_speed, wind_direction
    wind_direction = float(request.form["direction"])
    wind_speed = float(request.form['power'])
    print(wind_direction, wind_speed)
    success = airsim_backend.set_wind(wind_direction, wind_speed)
    print("request status: ", success)
    return {"responds": 200 if success else 404}


@inspection_page_bp.route("/flow_map", methods=["GET"])
def compute_flow_map():
    global wind_speed, wind_direction
    flow_map, flow_data = windfarm.generate_flow_map(wind_speed=wind_speed, wind_direction=wind_direction)
    retval, buffer = cv2.imencode('.png', flow_map)
    png_as_text = base64.b64encode(buffer)
    page_info = {
        'flow_map': png_as_text.decode('utf-8')
    }
    return page_info


@inspection_page_bp.route("/airsim_takeoff", methods=["GET"])
def takeoff():
    success = airsim_backend.takeoff()
    return {"responds": 200 if success else 404}


@inspection_page_bp.route("/start_wind_sim", methods=["GET"])
def start_wind_sim():
    global wind_sim
    success = False
    if not wind_sim and not airsim_backend.offline_mode:
        wind_sim = True
        print("Wind sim is started")
        yourThread = threading.Timer(POOL_TIME, wind_sim_tread, ())
        yourThread.start()
        success = wind_sim
    return {"responds": 200 if success else 404}


@inspection_page_bp.route("/stop_wind_sim", methods=["GET"])
def stop_wind_sim():
    global wind_sim
    success = False
    if wind_sim and not airsim_backend.offline_mode:
        wind_sim = False
        print("Wind sim is stopped")
        success = not wind_sim
    return {"responds": 200 if success else 404}


def wind_sim_tread():
    global airsim_backend
    global yourThread, wind_sim
    global wind_direction, wind_speed, windfarm

    with dataLock:
        # Do your stuff with commonDataStruct Here
        x, y, z = airsim_backend.get_drone_position()
        if x is None:
            print("Sim failed")
            wind_sim = False

        ws, wd = windfarm.get_wind_at_pos(x, y, z)
        airsim_backend.set_wind(wd[0], ws[0])
    # Set the next thread to happen
    if wind_sim:
        yourThread = threading.Timer(POOL_TIME, wind_sim_tread, ())
        yourThread.start()
    else:
        print("Thread stopped")
