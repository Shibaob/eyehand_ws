<<<<<<< HEAD
# eyehand_ws
=======
# hand_eye_calibration

roslaunch panda_description description.launch

roslaunch easy_handeye hand_eye_calibration.launch

rosrurqt_tf_tree rqt_tf_tree


# camera calibration

roslaunch panda_description description.launch

rosrun camera_calibration cameracalibrator.py --size 7x6 --square 0.01 image:=/camera/rgb/image_raw

# grasp

roslaunch panda_description description.launch

roslaunch grasp_demo start_server.launch 
>>>>>>> Initial commit
