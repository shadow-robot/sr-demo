#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

try:
    import xml.etree.cElementTree as ET
except ImportError:
    import xml.etree.ElementTree as ET

from math import pi # 3.14

"""
This script creates a urdf model of a rope starting from 
an xml file where joints and links have been already defined.
The rope is composed of several cylinders (link) connected by joints (joint).
"""

# Rope description
rope_length = 0.300 # [m]

# Number of cylinders that compose the rope 
num_cyl = 20

# Link description

length = str(rope_length/num_cyl) # length of each cylinder [m]
radius = "0.005"  # radius of each cylinder [m]

length_cl = str( float(length) + 0.0001)
radius_cl = str( float(radius) + 0.0001)
mass =   "0.010"   # mass   of each cylinder [Kg]

# Joint description

x = "1 " # don't forget the space 
y = "0 " # don't forget the space
z = "0"
friction = "1.0"
damping = "2.0"

# Joint Limits

effort = "5" # [N] 
lower = str(-pi/6) # [rad] 
upper = str(pi/6) # [rad]
velocity = "6.28" # [rad/s]

# Open a pre-existing xml model of a link in order to modify it
link_tree = ET.ElementTree(file='xml/link_rope.xml')

# Fetch the root element
link_root = link_tree.getroot()

# Fetch the attribute of the link
link_name = link_root[0].get('name')

# Add n-link and n-1 joint.

for n in range(1, num_cyl):
    
    # This step allows to make a copy of of link/joint  
    # as "separate" child, meaning that is possible
    # to modify the attribute of each link/joint 
    # separately  
    tree = ET.ElementTree(file='xml/link_rope.xml')
    joint_tree = ET.ElementTree(file='xml/joint_rope.xml')

    # Fetch the root element
    root = tree.getroot()
    joint_root = joint_tree.getroot()
    
    # Append a copy of a Link
    link_root.append(root[0])
    link_root.append(root[1])
    
    # Append a copy of a joint
    link_root.append(joint_root[0])

# Set the name of links and joints
number = 1

for elem in link_tree.iter(tag='link'):
    elem.set('name', link_name + str(number))
    number +=1     
else: number = 1
    
for elem in link_tree.iter(tag='gazebo'):
    elem.set('reference', link_name + str(number))
    number +=1
else: number = 1

for elem in link_tree.iter(tag='joint'):
    elem.set('name', "j_" + str(number))
    number +=1
else: number = 1

for elem in link_tree.iterfind('joint/parent'):
    elem.set('link', link_name + str(number))
    number +=1
else: number = 2

for elem in link_tree.iterfind('joint/child'):
    elem.set('link', link_name + str(number))
    number +=1

# Set the parameters

for elem in link_tree.iter(tag='cylinder'):
    elem.set('length',length)
    elem.set('radius',radius)
    #print elem.tag, elem.attrib

for elem in link_tree.iterfind('link/collision/geometry/cylinder'):
    elem.set('length',length_cl)
    elem.set('radius',radius_cl)
    #print elem.tag, elem.attrib    

for elem in link_tree.iter(tag='origin'):
    elem.set('xyz',"0 0 "+length)
    #print elem.tag, elem.attrib    

for elem in link_tree.iter(tag='axis'):
    elem.set('xyz',x+y+z)
    #print elem.tag, elem.attrib

for elem in link_tree.iter(tag='dynamics'):
    elem.set('damping', damping)
    elem.set('friction', friction)
    #print elem.tag, elem.attrib

link_tree.write("urdf/rope.urdf", 
                 xml_declaration=True, encoding='utf-8')



