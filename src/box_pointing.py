import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PointStamped, Vector3Stamped, Vector3

from rosprolog_client import Prolog, PrologException
from json_prolog_msgs.srv import PrologQueryRequest

from giskardpy.python_interface import GiskardWrapper

import numpy as np

class KnowRobWrapper:
	def __init__(self) -> None:
		self.prolog = Prolog()

	def query_knowrob(self, query_req: PrologQueryRequest) -> dict:
		#query_client = rospy.ServiceProxy('/rosprolog/query', PrologQuery)
		#query_response = query_client(query_req.mode, query_req.id, query_req.query)
		query_response = self.prolog.all_solutions(query_req.query)
		# print(f'Result:{query_response}')
		return query_response[0]


class GiskardHelper:
	def __init__(self):
		self.giskard = GiskardWrapper()

	def add_object(self, name:str, size:tuple, pose:PoseStamped, parent:str, type:str):
		if type=="box":
			self.giskard.add_box(name=name, size=size, pose=pose, parent_link=parent)

		elif type=="sphere":
			self.giskard.add_sphere(name=name, radius=size[0], pose=pose, parent_link=parent)

	def add_pointing_constraints(self, goal_point: PointStamped, tip: str,
								 pointing_axis: Vector3Stamped, root: str):
		self.giskard.set_pointing_goal(goal_point, tip_link=tip, pointing_axis=pointing_axis,
									   root_link=root)

	def execute_goal(self):
		resp = self.giskard.plan_and_execute()
		if resp.error_codes == 0:
			print("goal succeeds")
		else:
			print(f'code:{resp.error_codes}, response:{resp.error_messages}')


def create_sphere(translation: list, rotation:list, object:str,parent="map", radius=0.05, color=[0.5,0.5,0.5]):
	return "tell([is_physical_object({}), has_type(Shape, soma:'Shape'), has_type(ShapeRegion, soma:'SphereShape'),".format(object)+\
			"triple({},soma:hasShape,Shape), triple(Shape,dul:hasRegion,ShapeRegion), triple(ShapeRegion, soma:hasRadius, {}),".format(object, radius)+\
			"has_type(ColorType, soma:'Color'),"+\
			"triple({},soma:hasColor,ColorType),".format(object)+\
			"object_color_rgb({}, [{}, {}, {}]),".format(object, color[0], color[1], color[2])+\
			"is_at({}, [{}, [{}, {} ,{}], [{},{},{},{}]])]).".format(object, parent, translation[0], translation[1], translation[2], rotation[0],
																	  rotation[1], rotation[2], rotation[3])


def create_box(translation, rotation, object, parent="map", length=0.05, width=0.05, height=0.05, color=[0.5,0.5,0.5]):
	return "tell([is_physical_object({}),".format(object)+\
	"has_type(ColorType, soma:'Color'),"+\
	"triple({},soma:hasColor,ColorType),".format(object)+\
	"object_color_rgb({}, [{}, {}, {}]),".format(object, color[0], color[1], color[2])+\
	"has_type(Shape, soma:'Shape'), triple({},soma:hasShape,Shape), has_type(ShapeRegion, soma:'BoxShape'),".format(object)+\
	"triple(ShapeRegion, soma:hasLength, {}), triple(ShapeRegion, soma:hasWidth, {}), triple(ShapeRegion, soma:hasHeight, {}),".format(length, width, height)+\
	"is_at({}, [{}, [{}, {} ,{}], [{},{},{},{}]])]).".format(object, parent, translation[0], translation[1], translation[2], rotation[0], rotation[1],
															  rotation[2], rotation[3])

def rand_generator(high=99999999) -> int:
	rng = np.random.default_rng()
	low=0
	return rng.integers(low, high)


def kb_pose_to_poseStamped(pose) -> tuple:
	obj_pose = PoseStamped()
	obj_pose.header.frame_id = pose[0]
	obj_pose.pose.position = Point(pose[1][0], pose[1][1], pose[1][2])
	obj_pose.pose.orientation = Quaternion(pose[2][0], pose[2][1], pose[2][2], pose[2][3])
	print(f'P: {obj_pose.pose.position}, O: {obj_pose.pose.orientation}')
	return tuple([obj_pose.header.frame_id, obj_pose])


if __name__ == "__main__":
	""" Initial steps towards reasoning for Giskard"""
	rospy.init_node("query_node")

	knowrob = KnowRobWrapper()
	giskard = GiskardHelper()

	req = PrologQueryRequest()
	req.mode = 1
	# Create box
	box_dimensions = tuple([0.05, 0.05, 0.05])
	box_pose = PoseStamped()
	box_pose.header.frame_id = 'map'
	box_pose.pose.position = Point(0.5, 0.5, 1.0)
	box_pose.pose.orientation = Quaternion(0,0,0,1)

	object="O"
	req.id = str(rand_generator())
	req.query = create_box([box_pose.pose.position.x,
							box_pose.pose.position.y,
							box_pose.pose.position.z],
							[box_pose.pose.orientation.x,
							box_pose.pose.orientation.y,
							box_pose.pose.orientation.z,
							box_pose.pose.orientation.w], object)
	"""    print(req.query)
	box_object = knowrob.query_knowrob(req)[object]
	box_frame = box_object.split('#')[1]

	# Create sphere
	sphere_dimensions = tuple([0.05, 0.05, 0.05])
	sphere_pose = PoseStamped()
	sphere_pose.header.frame_id = 'map'
	sphere_pose.pose.position = Point(0.8,0,1)
	sphere_pose.pose.orientation = Quaternion(0,0,0,1)

	req.id = str(rand_generator())
	req.query = create_sphere([sphere_pose.pose.position.x, sphere_pose.pose.position.y,
								sphere_pose.pose.position.z],
							   [sphere_pose.pose.orientation.x,
								sphere_pose.pose.orientation.y,
								sphere_pose.pose.orientation.z,
								sphere_pose.pose.orientation.w], object)
	print(req.query)
	sphere_object = knowrob.query_knowrob(req)[object]
	sphere_frame = sphere_object.split('#')[1]

	giskard.add_object(name=box_frame, size=box_dimensions,
					pose=box_pose, parent='map', type="box")

	giskard.add_object(name=sphere_frame, size=tuple(0.05),
					pose=sphere_pose, parent='map', type="sphere") """

	# print(sphere_object)

	# Add pointing to sphere
	# get sphere object
	req.id = str(rand_generator())
	sphere_obj = "Obj"
	req.query = "is_physical_object({}), triple({}, soma:hasShape, Shape),".format(sphere_obj, sphere_obj)+\
		"triple(Shape, dul:hasRegion, ShapeRegion), has_type(ShapeRegion, soma:'SphereShape'),"+\
		"is_at({}, Pose).".format(sphere_obj)
	query_resp = knowrob.query_knowrob(req)
	object_to_point = query_resp[sphere_obj]
	(parent, object_pose) = kb_pose_to_poseStamped(query_resp['Pose'])

	goal = PointStamped()
	goal.point = object_pose.pose.position

	tip = 'l_gripper_tool_frame'
	pointing_along = Vector3Stamped()
	pointing_along.vector = Vector3(1, 0 ,0)

	giskard.add_pointing_constraints(goal_point= goal, tip=tip, 
								 pointing_axis= pointing_along, root='map')
	giskard.execute_goal()