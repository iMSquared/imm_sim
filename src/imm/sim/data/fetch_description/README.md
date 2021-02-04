## FETCH Description

Adaptation of `models_robot/fetch_description` from `roboschool:roboschool`

Which has been in turn adapted from the original file from `fetchrobotics:fetch_ros`.

In this repo the model has been adapted as follows:

* Restored `base_link` inertia properties
* Included valid inertia properties for virtual(massless) links
* Made the inertia properties valid for grippers, which violated triangle inequality.

Including the below text from the `roboschool` repository as reference:

### FETCH_MODIFICATIONS.md

Original code taken from:

https://github.com/fetchrobotics/fetch_ros/fetch_description

Modifications:

 * Comment out `bellows_link` and `bellows_joint`. You can push off the ground using this strange thing!

 * Set moments of inertia of `r_gripper_finger_link` and `l_gripper_finger_link` to non-zero.

 * Set moments of inertia of `base_link` to zero, except for Z-axis, to disable rotation other than around Z axis.

 * Added `<gazebo reference="head_camera_rgb_optical_frame">` for camera, turn `head_camera_rgb_optical_frame` 180 degrees as camera was facing backwards.

 * Spheres are now collision shape for wheels (fixes jumping).

 * Boxes for fingers collision shape.
