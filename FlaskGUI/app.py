import cv2
import numpy as np
from flask import Flask, url_for, render_template, Response, request, redirect

from backend.airsim_backend import AirSimBackend
from backend.input import RenderVideoInput

app = Flask(__name__)
video_input = RenderVideoInput(camera_index=0, sleep_time=0, async_stream=False, render=True, verbose=True)
airsim_backend = AirSimBackend()
mission_info_dict = {"target": ""}


@app.route('/')
def home_page():
    return render_template('index.html')


@app.route('/about')
def about_page():
    return render_template("about.html")


@app.route('/inspection')
def inspection_page():
    canvas_size = (800, 600)
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

    return render_template("inspection.html", data=data, canvas_size=canvas_size)


# We need to have all the variables enclosed with brackets <> but the : could be anything
@app.route('/dynamic_var/<dyn_var>:<dyn_var2>')
def dynamic_variable_example(dyn_var, dyn_var2):
    return f'This is a dynamic variable given through the url: {dyn_var} and {dyn_var2}'


@app.route('/video_feed')
def video_feed():
    return Response(video_input.genFrame(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/toggle_feed', methods=["POST"])
def toggle_feed():
    # video_input.render = not video_input.render
    print(f"TOGGLED to {video_input.render}")
    distance = float(request.form["test"])
    return ('', 204)


@app.route("/inspection/mission_info", methods=["POST", "GET"])
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
            return ("No target found", 204)


@app.route("/inspection/update_backend/<object_name>", methods=["POST"])
def update_backend(object_name):
    wind_turbines = airsim_backend.objects
    if object_name in wind_turbines:
        wind_turbines[object_name]["state"] = request.form["state"]
        wind_turbines[object_name]["height"] = request.form["turbine_height"]
        wind_turbines[object_name]["blade_radius"] = request.form["blade_radius"]
        mission_info_dict["target"] = object_name
    return ("updated", 204)


# Do GET stuff


# def gen_frames():
#     while True:
#         success, frame = camera.read()  # read the camera frame
#         if not success:
#             break
#         else:
#             frame = cv2.flip(frame, 1)
#             ret, buffer = cv2.imencode('.jpg', frame)
#             frame = buffer.tobytes()
#             yield (b'--frame\r\n'
#                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result


if __name__ == '__main__':
    app.run(debug=True)
