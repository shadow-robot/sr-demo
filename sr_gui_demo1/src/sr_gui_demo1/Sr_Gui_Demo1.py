#!/usr/bin/env python
#
# Copyright 2012 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import os
import roslib
roslib.load_manifest('sr_gui_demo1')
import rospy
import roslaunch
import rosparam

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

from QtCore import QEvent, QObject, Qt, QTimer, Slot
from QtGui import QShortcut, QMessageBox, QWidget, QIcon

class SrGuiCountingDemo(Plugin):

    def __init__(self, context):
        super(SrGuiCountingDemo, self).__init__(context)
        
        # Give QObjects reasonable names
        self.setObjectName('SrGuiCountingDemo')

        # Create QWidget
        self._widget = QWidget()
        
        # Get path to UI file which is a sibling of this file
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../../uis/sr_counting_demo.ui')
        
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        
        # Give QObjects reasonable names
        self._widget.setObjectName('SrCountingDemoUi')
        
        # Add widget to the user interface
        context.add_widget(self._widget)

        #Setting the initial state of the controller buttons
        self._widget.btn_1.setEnabled(False)
        self._widget.btn_2.setEnabled(False)
        self._widget.btn_3.setEnabled(False)
        self._widget.btn_4.setEnabled(False)
        self._widget.btn_5.setEnabled(False)
 
        # Attaching the button press event to their actions
        self._widget.play.pressed.connect(self.start_demo) 
        self._widget.btn_1.pressed.connect(self.number_one_clicked)
        self._widget.btn_2.pressed.connect(self.number_two_clicked)
        self._widget.btn_3.pressed.connect(self.number_three_clicked)
        self._widget.btn_4.pressed.connect(self.number_four_clicked)
        self._widget.btn_5.pressed.connect(self.number_five_clicked)
              
    def start_demo(self):
        """
        This function allows to:
  
        1. run the action server node

        2. run the get_joint_state node 

        3. load some parameters
        """
        newpid = os.fork()
        if(newpid > 0):
            os.system("roslaunch sr_counting_demo gazebo_sr_counting_demo_gui.launch")        
            os._exit(0)
        
        self._widget.play.setEnabled(False)
        self._widget.btn_1.setEnabled(True)
        self._widget.btn_2.setEnabled(True)
        self._widget.btn_3.setEnabled(True)
        self._widget.btn_4.setEnabled(True)
        self._widget.btn_5.setEnabled(True) 
        
    def number_one_clicked(self):
        # send goal = 1 to the action server
        newpid = os.fork()
        if(newpid > 0):
            os.system("roslaunch sr_counting_demo gazebo_sr_counting_demo_gui_client.launch goal:=1")
            os._exit(0)

    def number_two_clicked(self):
        # send goal = 2 to the action server
        newpid = os.fork()
        if(newpid > 0):
            os.system("roslaunch sr_counting_demo gazebo_sr_counting_demo_gui_client.launch goal:=2")
            os._exit(0)
        
    def number_three_clicked(self):
        # send goal = 3 to the action server
        newpid = os.fork()
        if(newpid > 0):
            os.system("roslaunch sr_counting_demo gazebo_sr_counting_demo_gui_client.launch goal:=3")
            os._exit(0)
       
    def number_four_clicked(self):
        # send goal = 4 to the action server
        newpid = os.fork()
        if(newpid > 0):
            os.system("roslaunch sr_counting_demo gazebo_sr_counting_demo_gui_client.launch goal:=4")
            os._exit(0)
        
    def number_five_clicked(self):
        # send goal = 5 to the action server
        newpid = os.fork()
        if(newpid > 0):
            os.system("roslaunch sr_counting_demo gazebo_sr_counting_demo_gui_client.launch goal:=5")
            os._exit(0)
        
    def _unregisterPublisher(self):
        pass    
    

    def shutdown_plugin(self):
        pass
        #newpid = os.fork()
        #if(newpid > 0):
        #   os.system("rosnode kill get_joint_state_service counter_server_py")
        #  os._exit(0)
        

    
