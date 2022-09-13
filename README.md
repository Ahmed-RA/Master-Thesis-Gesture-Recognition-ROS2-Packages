# Master-Thesis-Gesture-Recognition-ROS2-Packages
ROS2 Packages for Fast-Point-Feature-Histogram Gesture Recognition Implementation

# Hardware Stack
- PC running Ubuntu 20.04.3 LTS
- Orbbec Astra 3D Depth Camera (Drivers for interfacing the camera with openNI can be downloaded [here](https://astra-wiki.readthedocs.io/en/latest/downloadDriver.html#linux))

# Software Stack
- Python 3.8.10
- TensorFlow 2.9.1
- Numpy 1.22.4
- Open3D 0.15.2
- OpenNI Python Library 2.3.0
- ROS2 Foxy (Documentation can be found [here](https://docs.ros.org/en/foxy/))

# How to Run
- After setting up the software stack listed above, create a ROS2 workspace. Detailed guide on creating a ROS2 workspace can be found [here](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).
- Clone this repository into the newly created ROS2 workspace.
- Add the following alias to your `~/.bashrc` file to source ROS2 whenever needed (Assuming workspace is created in `~/workspace_1`): 
```
alias sros2="cd ~/workspace_1 && . install/setup.bash"
```
- Modify the `dist` variable in the `fpfh_publisher.py` python file (found in the `fpfh_pub` package) to your `Redist` folder of the OpenNI driver.
- Build the ROS2 packages with the following commands:
```
sros2
colcon build
```
- Run the point cloud processing node by opening a new terminal window and entering the following commands:
```
sros2
ros2 run fpfh_pub fpfh_publisher
```
A new window for visualizing input point clouds should appear, and process information should be displayed on the terminal.

- Next, open a another terminal and launch the classifier nodes:
```
sros2
ros2 launch fpfh_classifier_aggregator classifiers_launch.py
```
- Lastly, open another terminal window and run the aggregator node:
```
sros2
ros2 run fpfh_classifier_aggregator lstm_aggregator
```
Prediction results should be displayed on the terminal after a few seconds.
