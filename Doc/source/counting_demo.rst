Counting demo
=========================
This demo will make count the hand a number between one and five. At the beginning of the demo, the hand assumes the form of a fist
and then it starts to count lifting up one by one its fingers up to the goal number (previously selected by the user).

.. figure:: images/sr_counting_demo/2.png
   :scale: 80%
   :align: center

   *Demo snapshot: number two.*
 
Nodes
-------------------------

This demo has been implemented using the `actionlib <http://www.ros.org/wiki/actionlib protocol>`_ ros package. Such a package allows for the use of preemptible tasks. The (action) client and server communicate over a set of topics, described in the actionlib protocol.


Counter server
+++++++++++++++++++++++++
.. automodule:: counter_server
.. autoclass:: CounterDemoAction
   :members:

Counter client
+++++++++++++++++++++++++

This node acts as a client for the counter_server node. In case the counter_server is not running the client waits for it, otherwise it sends the number to be counted to the server. When this node is launched, it is asked to the user to select a number (between 1-5) from the keyboard.

.. automodule:: counter_client
.. autofunction:: counter_client
   
Utilities 
-------------------------
.. automodule:: sr_counting_demo_functions
.. autoclass:: CountingDemoFunctions
   :members:

How to run the demo in Gazebo
-------------------------------

To run the demo in `Gazebo <http://www.ros.org/wiki/simulator_gazebo/Tutorials>`_,
you need just to type::

    >> roscore

launch the gazebo simulator:: 

    >> roslaunch sr_hand gazebo_arm_and_hand.launch
    >> rosrun gazebo gui

and start the demo::

    >> roslaunch sr_counting_demo gazebo_sr_counting_demo.launch

Finally just follow the instructions on the console.








   

