#!/usr/bin/env python3
import os
import sys
from builtins import object

#from Cython.Includes.libcpp.iterator import insert_iterator
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates, ModelState
import time

home_directory = os.path.expanduser( '~' )
child_place = [-0.3,-1.2, 1.001373291015625]
hold_loc = [-0.3,1.2, 1.001373291015625]
locations = [1.25,1.234,0.2],[2.478,1.105,0.2],[-0.3,0.2,0.2],[3.177,-1.3,0.2],[1.29,-0.77,1],hold_loc
object_ind = {}
objects_dic = {'red' : ('red', home_directory + '/.gazebo/models/objects/red_ball.sdf'),
               'blue': ('blue', home_directory + '/.gazebo/models/objects/blue_ball.sdf'),
               'green':('green', home_directory + '/.gazebo/models/objects/green_ball.sdf'),
               'black':('black', home_directory + '/.gazebo/models/objects/black_ball.sdf'),
               'child':('child', home_directory + '/.gazebo/models/objects/blue_cube.sdf')}
objects = []

def initialize_environment():
    rospy.init_node('turtlebot3_ff', anonymous=True, log_level=rospy.WARN)

def spawn_model(object_name, spawn_location):
    #rospy.init_node('spawn_model', log_level=rospy.INFO)
    pose = Pose()
    pose.position.x = spawn_location[0]
    pose.position.y = spawn_location[1]
    pose.position.z = spawn_location[2]

    spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    spawn_model_client(model_name=object_name,
                       model_xml=open(objects_dic[object_name][1], 'r').read(),
                       robot_namespace='/stuff', initial_pose=pose, reference_frame='world')


def delete_model(name):
    # delete model
    srv = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    req = DeleteModelRequest()
    req.model_name = name
    resp = srv(req)


def create_scene(obj1,obj2,obj3,obj4):
    delete_model(obj1)
    delete_model(obj2)
    delete_model(obj3)
    delete_model(obj4)
    delete_model('child')
    time.sleep(.1)
    spawn_model(obj1, locations[0])
    spawn_model(obj2, locations[1])
    spawn_model(obj3, locations[2])
    spawn_model(obj4, locations[3])
    spawn_model('child', locations[4])

      
def goal_checker():
    pass
    #TODO		



def place(object_name, location_ind):
    print('place:{} {}'.format(object_name, location_ind))
    delete_model(object_name)
    location = None
    if location_ind == 4:
        location = child_place
    else:
        location = locations[location_ind]
    time.sleep(1)
    spawn_model(object_name=object_name, spawn_location=location)

def pick(object_name):
    global object_ind
    print("delete:{}".format(object_name))
    delete_model(object_name)
    time.sleep(1)
    print("before adding")
    try:
        spawn_model(object_name=object_name, spawn_location=hold_loc)
    except Exception as e:
        print(str(e))
    print("added:{}".format(object_name))
    
    
if __name__ == '__main__':
    try:
        command = sys.argv[1]
        print("------------------------- args:{}".format(sys.argv))
        if command == "init_env":

            create_scene(sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
        if command == "pick":
            print(sys.argv[2])
            pick(sys.argv[2])
        if command == "place":
            place(sys.argv[2], int(sys.argv[3]))
    except:
        print("there was an error")
	

