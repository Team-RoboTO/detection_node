
1. Install ROS2 foxy https://docs.ros.org/en/foxy/Installation.html
2. Install rclpy https://github.com/ros2/rclpy
3. git clone https://github.com/Team-RoboTO/detection_node
4. `$ cd detection_node/detection_node`
5. Install ultralytics (the library for YOLOv8) `$ pip install ultralytics`
6. In a terminal t1 launch the node that reads images from webcam and publishes them `$ cd detection_node/detection_node && python3 webcam.py`. It should automatically download a trained model
7. In a terminal t2 launch the detection node that takes the images, computes detection/tracking and applies the bounding box `$ cd detection_node/detection_node && python3 detector.py`
8. Visualize the predictions with `rivz2`: open `rviz2` from terminal t3, click `add` and then add a topic `image`/`bbox`
9. Pray that everything goes well
