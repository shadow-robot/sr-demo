#ORK Tabletop Object Recognition
This is launch, config and script files necessary to run a tabletop object recognition.

##Dependencies
1. Shadow robot ROS development environment, installed using the one-liner
2. ORK, [installed using the rosinstall method](http://wg-perception.github.io/object_recognition_core/install.html#rosinstall-file)
3. Shadow's tabletop detection Docker image (`docker pull shadowrobot/tabletop-detection`)
4. A Kinect V1 (XBox 360)

##Installation
1. `cd your_catkin_workspace/src`
2. `git clone git@github.com:shadow-robot/sr-demo.git`
3. `cd .. && catkin_make`

##Running
1. `docker run --name tabletop --rm --publish 5984:5984 shadowrobot/tabletop-detection`
2. `roslaunch sr_ork_tabletop ork_tabletop.launch`
