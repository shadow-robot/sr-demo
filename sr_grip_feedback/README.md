#Grip Feedback Demo
This is a simple demo of the lite hand's grip feedback data.

##Dependencies
1. Shadow robot ROS development environment, installed using the one-liner
2. A physical lite hand, connected via ethercat
3. [rqt_multiplot](https://github.com/ethz-asl/rqt_multiplot_plugin) and it's dependencies

##Installation
1. `cd your_catkin_workspace/src`
2. `git clone git@github.com:shadow-robot/sr-demo.git`
3. `cd .. && catkin_make`

##Running
1. Check out, make and source the `demohand_lite_v1` branch of `sr_config`
2. `roslaunch sr_ethercat_hand_config_sr_rhand.launch`
3. `roslaunch sr_grip_feedback sr_grip_feedback.launch`
5. In the multiplot window within rqt, browse to the included config (`perspectives/mulitplot/hand_feedback_demo.xml`)
