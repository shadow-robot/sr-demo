Frequently used commands
===========================

A list of useful and frequently used commands.

How to manage a workspace in ROS
---------------------------------
 
To create a new workspace::

    >> rosws init ~/my_workspace /opt/ros/fuerte

Such command basically makes two things:

1.  creates the following files: *.rosinstall*, *setup.bash*, *setup.sh* and *setup.zsh*

2.  adds to the workspace all the packages installed in /opt/ros/fuerte 
 
To use the created workspace::

    >> source ~/my_workspace/setup.bash

.. note:: If you have different workspaces what you need to do for changing workspace is just to re-source the desired workspace.
          
To add a new package to the workspace::

    >> rosws set ~/my_workspace/new_pkg 

Every time a new package is added, what you need to do is to update the *ROS_PACKAGE_PATH* so that the new package can be "seen" by ROS::

    >> source ~/my_workspace/setup.bash

To check whether the new package has been added to ROS_PACKAGE_PATH::

    >> echo $ROS_PACKAGE_PATH 

Ros common commands
----------------------
Some examples for:

1. creating a new package::

    >> roscreate-pkg new_pkg_name rospy sr_hand actionlib roslib roscpp std_msgs actionlib_msgs (dependencies)

2. Moving into a package directory::

    >> roscd sr_demo

3. viewing the list of active topics::

    >> rostopic list

4. showing the info about the workspace::

    >> rosws info

.. tip:: for more commands have a look at the :download:`ROS_cheatsheet...<pdf/ROScheatsheet.pdf>`


Urdf models
-----------------------------------------------------------
For creating an `urdf <http://www.ros.org/wiki/urdf/Tutorials>`_ model from a xacro file is sufficient type::

    >> rosrun xacro xacro.py my_dir/my_file.xacro > my_dir/my_file.urdf


Bzr
-----
How to use bazar in four steps:

1. Get the status::
    
    >> bzr status

2. Add new files::
    
    >> bzr qadd    

3. Commit changes::
    
    >> bzr qcommit

4. Push the changes on launchpad::

    >> bzr push --remember bzr+ssh://bazaar.launchpad.net/.../my_demo

.. note:: The use of the option *remember* allows then to type just::

    >> bzr push


.. tip:: for more commands have a look at the :download:`bzr_cheatsheet...<pdf/bzr-en-quick-reference.pdf>`












