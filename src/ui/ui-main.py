from flask import Flask, render_template, request, jsonify
import subprocess

app = Flask(__name__, static_folder='detections')
app.add_url_rule('/detections/<path:filename>', endpoint='detections', view_func=app.send_static_file)
@app.route('/')
def index():
    return render_template('index.html')   # loads UI

@app.route('/run_mapreader', methods=['POST'])
def run_mapreader():
    # Here you’d call your ROS2 mapReader code
    try:
        # Run ROS2 command
        # result = subprocess.run(
        #     ["ros2", "run", "mapReader", "map_reader"],
        #     capture_output=True,
        #     text=True,
        #     check=True
        # )
        subprocess.Popen(["ros2", "run", "mapReader", "map_reader"])
        return jsonify({"status": "success", "message": "Triggered mapReader"})
    except subprocess.CalledProcessError as e:
        return jsonify({"status": "error", "message": e.stderr})

@app.route('/view_detections')
def view_detections():
    # Display saved detection images
    import os
    image_folder = 'detections'
    images = [img for img in os.listdir(image_folder) if img.endswith(('.png', '.jpg', '.jpeg'))]
    image_paths = [os.path.join(image_folder, img) for img in images]
    return render_template('view_detections.html', images=image_paths)

if __name__ == '__main__':
    app.run(debug=True)
