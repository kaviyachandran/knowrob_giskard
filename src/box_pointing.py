import rospy

from rosprolog_client import Prolog, PrologException
from json_prolog_msgs.srv import PrologQuery, PrologQueryRequest, PrologQueryResponse

import numpy as np

class KnowRobWrapper:
    def __init__(self) -> None:
        self.prolog = Prolog()
        self.prolog_query = ""

    def query_knowrob(self, query_req: PrologQueryRequest) -> str:
        #query_client = rospy.ServiceProxy('/rosprolog/query', PrologQuery)
        #query_response = query_client(query_req.mode, query_req.id, query_req.query)
        query_response = self.prolog.all_solutions(query_req.query)
        # print(f'Result:{query_response}')
        return query_response[0]


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
    "has_type(Shape, soma:'Shape'), triple({},soma:hasShape,Shape),object_dimensions({}, {}, {}, {}),".format(object, object, length, width, height)+\
    "is_at({}, [{}, [{}, {} ,{}], [{},{},{},{}]])]).".format(object, parent, translation[0], translation[1], translation[2], rotation[0], rotation[1], 
                                                              rotation[2], rotation[3])

def rand_generator(high=99999999) -> int:
    rng = np.random.default_rng()
    low=0
    return rng.integers(low, high)


if __name__ == "__main__":
    """ Initial steps towards reasoning for Giskard"""
    rospy.init_node("query_node")
    
    knowrob = KnowRobWrapper()
    req = PrologQueryRequest()
    req.mode = 1
    # Create box
    object="O"
    req.id = str(rand_generator())
    req.query = create_box([0,0,0], [0,0,0,1], object)
    print(req.query)
    box_object = knowrob.query_knowrob(req)
    print(box_object[object])

    # Create sphere
    req.id = str(rand_generator())
    req.query = create_sphere([1,0,0], [0,0,0,1], object)
    print(req.query)
    sphere_object = knowrob.query_knowrob(req)
    print(sphere_object[object])