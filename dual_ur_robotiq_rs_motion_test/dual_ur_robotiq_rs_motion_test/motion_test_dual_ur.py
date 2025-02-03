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

def reorder_elements(line):
	"""
	Reorders elements in a specific way.
	Modify this function based on the required order.
	"""
	# Example: Assume each line has [x, y, z, roll, pitch, yaw]
	# and we want to reorder it to [yaw, pitch, roll, z, y, x]
	if line[1] < 0.0000001:
		gripper = 0.01
	else:
		gripper = 0.65
	return [line[3]*np.pi, line[2]*np.pi, line[0]*np.pi, line[4]*np.pi, line[5]*np.pi, line[6]*np.pi,
		 	 gripper, gripper, -gripper, -gripper, -gripper, gripper]  # Adjust as needed

def extend_shorter_list(list1, list2):
	"""
	Extends the shorter list by copying its last element until both lists have the same length.
	"""
	len1, len2 = len(list1), len(list2)

	if len1 < len2 and list1:
		list1.extend([list1[-1]] * (len2 - len1))  # Copy last element
	elif len2 < len1 and list2:
		list2.extend([list2[-1]] * (len1 - len2))

def get_path(scenario,seed):
	base_path = os.path.join(
		"/home/lee/parasol/Lazy-DaSH/experiment_ws/src/dual_ur_robotiq_rs_motion_test/path",
		str(scenario),
		"data",
	)
	# Initialize lists to store paths
	right_ur_path = []
	left_ur_path = []

	# Ensure the base path exists
	if not os.path.exists(base_path):
		raise FileNotFoundError(f"Directory {base_path} does not exist.")

	# Iterate over all files in the directory
	for file_name in os.listdir(base_path):
		file_path = os.path.join(base_path, file_name)
		if ("ur5e" not in file_name) or (seed not in file_name):
			print(file_path, "is not")
			continue
		print("selected:",file_path)

		# Ensure it's a file before processing
		if os.path.isfile(file_path):
			with open(file_path, 'r') as f:
				lines = [reorder_elements(tuple(map(float,line.strip().split()))) for line in f.readlines()]

			# Determine which UR5e arm the file belongs to
			if "ur5e_0" in file_name:
				right_ur_path = lines
			elif "ur5e_1" in file_name:
				left_ur_path = lines
	
	extend_shorter_list(left_ur_path, right_ur_path)

	return left_ur_path, right_ur_path

def main(args=None):
	logger.info(f'\n\n+++++++++++++++++++++++++ START +++++++++++++++++++++++++\n')

	time.sleep(10)

	moveit = MoveItPy(node_name="motion_test")

	left_ur_name = "left_ur_manipulator"
	right_ur_name = "right_ur_manipulator"
	left_gripper_name = "left_robotiq_2f_85_gripper"
	right_gripper_name = "right_robotiq_2f_85_gripper"

	left_ur = moveit.get_planning_component(left_ur_name)
	right_ur = moveit.get_planning_component(right_ur_name)
	left_gripper = moveit.get_planning_component(left_gripper_name)
	right_gripper = moveit.get_planning_component(right_gripper_name)

	left_ur.set_workspace(-3.0, -3.0, 0.0, 3.0, 3.0, 3.0)
	right_ur.set_workspace(-3.0, -3.0, 0.0, 3.0, 3.0, 3.0)

	time.sleep(3)

	left_ur_path, right_ur_path = get_path("sort_4objs","41620151")

	for i in range(len(left_ur_path)):
		if i == 0:
			continue

		# if i % 5 != 1:
		# 	continue
	
		logger.info(f'===============================================================================')
		# logger.info(f'Timestep: {i+1}/{len(left_ur_path)}')
		logger.info(f'Prev Timestep: {i} | Left: {left_ur_path[i-1][-1]}, Right: {right_ur_path[i-1][-1]}')
		logger.info(f'Curr Timestep: {i+1} | Left: {left_ur_path[i][-1]}, Right: {right_ur_path[i][-1]}')
		
		next_config_left_ur = left_ur_path[i][:6]
		next_config_right_ur = right_ur_path[i][:6]
		next_config_left_gripper = left_ur_path[i][6:]
		next_config_right_gripper = right_ur_path[i][6:]
		
		logger.info(f'Plan Left')
		current_config_left_ur = None
		current_config_left_gripper = None
		with moveit.get_planning_scene_monitor().read_only() as scene:
			robot_state_scene = scene.current_state
			current_config_left_ur = robot_state_scene.get_joint_group_positions(left_ur_name)
			current_config_left_gripper = robot_state_scene.get_joint_group_positions(left_gripper_name)


		logger.info(f'Plan Left Manipulator')
		robot_state = RobotState(moveit.get_robot_model())
		robot_state.set_joint_group_positions(left_ur_name, current_config_left_ur)
		left_ur.set_start_state(robot_state=robot_state)
		robot_state.set_joint_group_positions(left_ur_name, next_config_left_ur)
		left_ur.set_goal_state(robot_state=robot_state)

		_ = plan_and_execute(moveit, left_ur)

		logger.info(f'Plan Left Gripper')
		robot_state = RobotState(moveit.get_robot_model())
		robot_state.set_joint_group_positions(left_gripper_name, current_config_left_gripper)
		left_gripper.set_start_state(robot_state=robot_state)
		robot_state.set_joint_group_positions(left_gripper_name, next_config_left_gripper)
		left_gripper.set_goal_state(robot_state=robot_state)

		_ = plan_and_execute(moveit, left_gripper)
	
	
		logger.info(f'Plan Right')
		current_config_right_ur = None
		current_config_right_gripper = None
		with moveit.get_planning_scene_monitor().read_only() as scene:
			robot_state_scene = scene.current_state
			current_config_right_ur = robot_state_scene.get_joint_group_positions(right_ur_name)
			current_config_right_gripper = robot_state_scene.get_joint_group_positions(right_gripper_name)

		logger.info(f'Plan Right Manipulator')
		robot_state = RobotState(moveit.get_robot_model())
		robot_state.set_joint_group_positions(right_ur_name, current_config_right_ur)
		right_ur.set_start_state(robot_state=robot_state)
		robot_state.set_joint_group_positions(right_ur_name, next_config_right_ur)
		right_ur.set_goal_state(robot_state=robot_state)

		_ = plan_and_execute(moveit, right_ur)

		logger.info(f'Plan Right Gripper')
		robot_state = RobotState(moveit.get_robot_model())
		robot_state.set_joint_group_positions(right_gripper_name, current_config_right_gripper)
		right_gripper.set_start_state(robot_state=robot_state)
		robot_state.set_joint_group_positions(right_gripper_name, next_config_right_gripper)
		right_gripper.set_goal_state(robot_state=robot_state)

		_ = plan_and_execute(moveit, right_gripper)



	logger.info(f'+++ Shutting Down +++')
	moveit.shutdown()


if __name__ == "__main__":
	main()