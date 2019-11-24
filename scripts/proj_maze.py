#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Chirav Dave"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import numpy as np
import os, sys
class Maze:

	def copy_empty_world(self):
		parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
		#File containing empty world description
		f_in = open(parent_dir+'/worlds/empty_world.sdf', 'r')
		#File to save your maze
		f_out = open(parent_dir+'/worlds/maze.sdf', 'w')
		#Copying empty world description into output maze
		for line in f_in:
			f_out.write(line)
		f_in.close()
		return f_out

	#Method to add wall description surrounding the maze
	def add_walls_description(self, f_out):
		for i in range(1, 5):
			if(i%2==0):
				f_out.write('<model name=\'wall{}\'>\n'.format(i))
				f_out.write('<static>1</static>\n<link name=\'link1\'>\n<pose frame=\'\'>0 0 0.42 0 -0 0</pose>\n<collision name=\'collision\'>\n<geometry>\n<box>\n<size>6 0.01 0.01</size>\n</box>\n')
				f_out.write('</geometry>\n<max_contacts>10</max_contacts>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<torsional>\n<ode/>\n</torsional>\n<ode/>\n</friction>\n</surface>\n</collision>\n')
				f_out.write('<visual name=\'visual\'>\n<cast_shadows>0</cast_shadows>\n<geometry>\n<box>\n<size>6 0.01 0.01</size>\n</box>\n</geometry>\n<material>\n<script>\n')
				f_out.write('<uri>model://grey_wall/materials/scripts</uri>\n<uri>model://grey_wall/materials/textures</uri>\n<name>vrc/grey_wall</name>\n</script>\n</material>\n</visual>\n<self_collide>0</self_collide>\n')
				f_out.write('<kinematic>0</kinematic>\n<gravity>1</gravity>\n</link>\n<pose frame=\'\'>-0.779308 4.01849 0 0 -0 0</pose>\n</model>\n')
			else:
				f_out.write('<model name=\'wall{}\'>\n'.format(i))
				f_out.write('<static>1</static>\n<link name=\'link\'>\n<pose frame=\'\'>0 0 0.42 0 -0 0</pose>\n<collision name=\'collision\'>\n<geometry>\n<box>\n<size>4 0.01 0.01</size>\n</box>\n')
				f_out.write('</geometry>\n<max_contacts>10</max_contacts>\n<surface>\n<contact>\n<ode/>\n</contact>\n<bounce/>\n<friction>\n<torsional>\n<ode/>\n</torsional>\n<ode/>\n</friction>\n</surface>\n</collision>\n')
				f_out.write('<visual name=\'visual\'>\n<cast_shadows>0</cast_shadows>\n<geometry>\n<box>\n<size>4 0.01 0.01</size>\n</box>\n</geometry>\n<material>\n<script>\n')
				f_out.write('<uri>model://grey_wall/materials/scripts</uri>\n<uri>model://grey_wall/materials/textures</uri>\n<name>vrc/grey_wall</name>\n</script>\n</material>\n</visual>\n<self_collide>0</self_collide>\n')
				f_out.write('<kinematic>0</kinematic>\n<gravity>1</gravity>\n</link>\n<pose frame=\'\'>-0.779308 4.01849 0 0 -0 0</pose>\n</model>\n')

	#Method to place walls around the maze
	def add_walls(self, f_out):
		scale = (4+2)/5
		wall_dimensions = [(0, 2, -1.55905, scale, 1), (3, 0, 0, scale, 1), (6, 2, -1.55905, scale, 1), (3, 4, 0, scale, 1)]
		for i in range(4):
			f_out.write('<model name=\'wall{}\'>\n'.format(i+1))
			f_out.write('<pose frame=\'\'>{} {} 0 0 -0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1], wall_dimensions[i][2]))
			f_out.write('<scale>{} {} 0.03</scale>\n'.format(wall_dimensions[i][3], wall_dimensions[i][4]))
			if(i%2==0):
				f_out.write('<link name=\'link\'>\n')
				f_out.write('<pose frame=\'\'>{} {} 0.42 0 -0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1], wall_dimensions[i][2]))
				f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 0 0 -0 0</acceleration>\n<wrench>0 0 0 0 -0 0</wrench>\n</link>\n</model>\n')

			else:
				f_out.write('<link name=\'link1\'>\n')
				f_out.write('<pose frame=\'\'>{} {} 0.42 0 -0 {}</pose>\n'.format(wall_dimensions[i][0], wall_dimensions[i][1], wall_dimensions[i][2]))
				f_out.write('<velocity>0 0 0 0 -0 0</velocity>\n<acceleration>0 0 0 0 -0 0</acceleration>\n<wrench>0 0 0 0 -0 0</wrench>\n</link>\n</model>\n')


	#Method to generate maze
	def generate_maze(self,  scale=1):
		#np.random.seed(seed)
		
		f_out = self.copy_empty_world()
		self.add_walls(f_out)
		

		f_out.write('</state>')
		#self.add_goal_description(f_out, grid_dimension*0.5)
		self.add_walls_description(f_out)
		#self.add_can_description(f_out, coords)
		f_out.write('</world>\n</sdf>')
		f_out.close()
		maze = [0, 0, 'EAST', scale]
		return maze
