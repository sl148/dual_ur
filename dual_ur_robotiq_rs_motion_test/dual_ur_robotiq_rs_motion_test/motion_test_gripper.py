import os
import time
import rclpy
from rclpy.logging import get_logger
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from moveit.core.robot_state import RobotState
from moveit.planning import (
	MoveItPy,
	MultiPipelinePlanRequestParameters,
)

from rclpy.impl import rcutils_logger
logger = rcutils_logger.RcutilsLogger(name="env_gen")

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import numpy as np
import pickle
from pathlib import Path

import time

np.random.seed(217)

def plan_and_execute(
	robot,
	planning_component,
	single_plan_parameters=None,
	multi_plan_parameters=None,
	sleep_time=0.0,
	):
	"""A helper function to plan and execute a motion."""
	# plan to goal
	logger.info("Planning trajectory")
	if multi_plan_parameters is not None:
		plan_result = planning_component.plan(
			multi_plan_parameters=multi_plan_parameters
		)
	elif single_plan_parameters is not None:
		plan_result = planning_component.plan(
			single_plan_parameters=single_plan_parameters
		)
	else:
		plan_result = planning_component.plan()

	# execute the plan
	if plan_result:
		logger.info("Executing plan")
		robot_trajectory = plan_result.trajectory
		print(len(robot_trajectory))
		robot.execute(robot_trajectory, controllers=[])
	else:
		logger.error("Planning failed")
		return False

	time.sleep(sleep_time)
	return True
	
def plan_traj(
	robot,
	planning_component,
	single_plan_parameters=None,
	multi_plan_parameters=None,
	sleep_time=0.0,
	):
	"""A helper function to plan and execute a motion."""
	# plan to goal
	logger.info("Planning trajectory")
	if multi_plan_parameters is not None:
		plan_result = planning_component.plan(
			multi_plan_parameters=multi_plan_parameters
		)
	elif single_plan_parameters is not None:
		plan_result = planning_component.plan(
			single_plan_parameters=single_plan_parameters
		)
	else:
		plan_result = planning_component.plan()

	if plan_result:
		return plan_result.trajectory
	else:
		return None

def check_overlap(new_pos, new_size, existing_objects):
	new_pos_array = np.array(new_pos)
	new_size_array = np.array(new_size)

	# Check if any part of the new object is closer than 0.1 to the origin
	if np.any(np.abs(new_pos_array[:2]) - new_size_array[:2]/2 < 0.1):
		return True

	for obj in existing_objects:
		pos, size = obj
		pos_array = np.array(pos)
		size_array = np.array(size)
		if np.all(np.abs(new_pos_array - pos_array) < (new_size_array + size_array) / 2):
			return True
	return False

def generate_random_pose_obj(object_size, scale):
	size_array = np.array(object_size)
	while True:
		x = np.random.uniform(-0.7, 0.7)*scale
		y = np.random.uniform(-0.7, 0.7)*scale
		z = np.random.uniform(0.0, 0.7)*scale
		position = np.array([x, y, z])
		logger.info(f'1 {position}')
		z = size_array[2]/2.
		position = np.array([x, y, z])
		logger.info(f'2 {position}')
	
		# Check if any part of the object is closer than 0.1 to the origin
		if np.all(np.abs(position)[:2] - size_array[:2]/2 >= 0.1):
			return position.tolist()

def setup_workspace(moveit,scale):
	existing_objects = []
	with moveit.get_planning_scene_monitor().read_write() as scene:
		logger.info(f"\tRemoving all existing objects")
		scene.remove_all_collision_objects()
	
	with moveit.get_planning_scene_monitor().read_write() as scene:
		objects_to_add = [
			("monitor", [0.5, 0.3, 0.05]),
			("desktop", [0.4, 0.4, 0.4]),
			("screwdriver_box", [0.1, 0.2, 0.1]),
			("container", [0.3, 0.3, 0.2])
		]
	
		for name, size in objects_to_add:
			max_attempts = 100000
			for _ in range(max_attempts):
				position = generate_random_pose_obj(size,scale)
				if not check_overlap(position, size, existing_objects):
					add_box(scene, name, size, position)
					existing_objects.append((position, size))
					logger.info(f"\tSuccessful placement of {name}")
					break
				else:
					logger.warn(f"Failed to place {name} without overlap after {max_attempts} attempts. Existing objects {existing_objects}")
			
		base_position = [0.0, 0.0, -0.11]
		base_size = [2.0, 2.0, 0.2]
		add_box(scene, "base", base_size, base_position, base=True)
		existing_objects.append((base_position, base_size))
		
		scene.current_state.update()
	
	return existing_objects

def add_box(scene, name, size, position, base=False):
	co = CollisionObject()
	co.header.frame_id = 'world'
	co.id = name
	box = SolidPrimitive()
	box.type = SolidPrimitive.BOX
	box.dimensions = size
	co.primitives.append(box)
	pose = Pose()
	pose.position.x, pose.position.y, pose.position.z = position
	co.primitive_poses.append(pose)
	co.operation = CollisionObject.ADD
	scene.apply_collision_object(co)

def check_valid_state_joints(moveit, robot_name, joint_angles):
	with moveit.get_planning_scene_monitor().read_only() as scene:
		robot_state = scene.current_state
		original_joint_positions = robot_state.get_joint_group_positions(robot_name)
		
		robot_state.set_joint_group_positions(robot_name, joint_angles)
		robot_state.update()  # required to update transforms
		robot_collision_status = scene.is_state_colliding(
			robot_state=robot_state, joint_model_group_name=robot_name, verbose=False
		)
		robot_collision_status &= scene.is_state_valid(
			robot_state=robot_state, joint_model_group_name=robot_name, verbose=False
		)
		# logger.info(f"\nRobot is in collision: {robot_collision_status}\n")
		
		# Restore the original state
		robot_state.set_joint_group_positions(
			robot_name,
			original_joint_positions,
		)
		robot_state.update()  # required to update transforms
		return not robot_collision_status
		
def check_valid_state_pose(moveit, robot_name, pose):
	with moveit.get_planning_scene_monitor().read_only() as scene:
		robot_state = scene.current_state
		original_joint_positions = robot_state.get_joint_group_positions(robot_name)
		
		robot_state.set_from_ik(robot_name, vec_to_pose(pose), "tool0")
		robot_state.update()  # required to update transforms
		robot_collision_status = scene.is_state_colliding(
			robot_state=robot_state, joint_model_group_name=robot_name, verbose=False
		)
		robot_collision_status &= scene.is_state_valid(
			robot_state=robot_state, joint_model_group_name=robot_name, verbose=False
		)
		# logger.info(f"\nRobot is in collision: {robot_collision_status}\n")
		
		# Restore the original state
		robot_state.set_joint_group_positions(
			robot_name,
			original_joint_positions,
		)
		robot_state.update()  # required to update transforms
		return not robot_collision_status
	  

def generate_random_pose(scale):
	x = np.random.uniform(-0.7, 0.7)*scale
	y = np.random.uniform(-0.7, 0.7)*scale
	z = np.random.uniform(0.0, 0.7)*scale
	return [x, y, z]

def vec_to_pose(pose):
	pose_msg = Pose()
	pose_msg.position.x = pose[0]
	pose_msg.position.y = pose[1]
	pose_msg.position.z = pose[2]
	# pose_msg.orientation.w = 1.0
	pose_msg.orientation.w = 0.0
	pose_msg.orientation.x = 1.0
	pose_msg.orientation.y = 0.0
	pose_msg.orientation.z = 0.0
	return pose_msg

def define_scale(ur_type):
	if 'ur3' in ur_type:
		scale = 0.8
	elif 'ur5' in ur_type:
		scale = 1.
	elif 'ur10' in ur_type:
		scale = 1.2
	elif 'ur16' in ur_type:
		scale = 1.1
	elif 'ur20' in ur_type:
		scale = 1.4
	elif 'ur30' in ur_type:
		scale = 1.5
	return scale

def main(args=None):
	logger.info(f'\n\n+++++++++++++++++++++++++ START +++++++++++++++++++++++++\n')

	time.sleep(10)

	moveit = MoveItPy(node_name="motion_test")

	robot_name = "right_robotiq_2f_85_gripper"
	robot = moveit.get_planning_component(robot_name)
	robot.set_workspace(-3.0, -3.0, 0.0, 3.0, 3.0, 3.0)

	time.sleep(3)

	robot_model = moveit.get_robot_model()
	robot_state = RobotState(robot_model)

	num_timesteps = 5
	timestep = 0
	open = 0.01
	close = 0.8
	sequence = [open,close]
	index = 0
	while timestep < num_timesteps:
		logger.info(f'===============================================================================')
		logger.info(f'Timestep: {timestep}')
		
		target = None
		current = None
		with moveit.get_planning_scene_monitor().read_only() as scene:
			robot_state = scene.current_state
			current = robot_state.get_joint_group_positions(robot_name)
			target = [sequence[int(index%2)] for _ in current]
			logger.info(f"Current Joint Positions: {current}")
			logger.info(f"Current Target Positions: {target}")

		robot_state.set_joint_group_positions(robot_name, target)
		robot.set_goal_state(robot_state=robot_state)
		
		robot_state.set_joint_group_positions(robot_name, current)
		robot.set_start_state(robot_state=robot_state)
		
		result = plan_and_execute(moveit, robot)
		if result:
			index += 1

		timestep += 1


	logger.info(f'+++ Shutting Down +++')
	moveit.shutdown()


if __name__ == "__main__":
	main()