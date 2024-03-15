#!/usr/bin/env python3
# license removed for brevity
# from https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
import os
import rospy
import actionlib
import time
import random
from future.standard_library import exclude_local_folder_imports
from geometry_msgs.msg import Point
from task4_env.srv import navigate, navigateResponse, pick, pickResponse, place, placeResponse, info, infoResponse  #
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
SUCCEEDED = 3
actions = []
observations = []
rewards = []
log = []
total_reward_msg = ""
state_toy_locations={"green":0, "blue":1, "black":2, "red":3}
state_locations_toy={0:"green", 1:"blue", 2:"black", 3:"red", 5:"None"}
state_toy_rewards={"green":15, "blue":20, "black":30, "red":40}
state_robot_location = 4
state_is_holding= False
state_holding_object_name = None
objects=["green", "blue", "black", "red"]
game_over = False
pick_calls=0
nav_calls=0
def get_state():
    return "state:[robot location:{} toys_location:{} locations_toy:{} toys_reward:{} holding_toy:{}]".format(state_robot_location, state_toy_locations, state_locations_toy, state_toy_rewards, state_is_holding)

def calc_rewards(): # fixed reward per toy
    global state_toy_rewards
    rewards=[15, 20, 30, 40]    
    a_reward = 0
    state_toy_rewards[objects[0]] = rewards[a_reward]
    b_reward = 1
    state_toy_rewards[objects[1]] = rewards[b_reward]
    c_reward = 2
    state_toy_rewards[objects[2]] = rewards[c_reward]
    d_reward = 3
    state_toy_rewards[objects[3]] = rewards[d_reward]
    print("rewards:")
    print(state_toy_rewards)



def calc_nav_succ(location): # navigate fails, the no motion is executed in gazebo (state does not change, nav counter +1)
    weightsSuccess = [0.8,0.85,0.9,0.75,1.0] # navigation success is dependent on the location
    weightsObs = [1.0,1.0,1.0,1.0,1.0] # nav success is always observed correctly	
    succ = random.random() <= weightsSuccess[location]
    obs = random.random() <= weightsObs[location]
    print("sampled navigate real obs:{}".format(obs))
    print("sampled navigate succ:{}".format(succ))
    if obs:
        return (succ,succ)
    else:
        return (succ, not succ)



def calc_locations(): # we can leave this, the toys get distributed to location according to the same backstory
    global state_locations_toy
    locations = {}
    weightsLocationA = [0.1,0.05,0.8,0.05]
    a_location = np.random.choice(4, 1, p=weightsLocationA)[0]
    locations[a_location]=objects[0]

    weightsLocationB = [0.7,0.1,0.1,0.1]
    selectedWeight = weightsLocationB[a_location]
    for i in range(4):
        weightsLocationB[i] += selectedWeight/3.0
    weightsLocationB[a_location]=0
    b_location = np.random.choice(4, 1, p=weightsLocationB)[0]
    locations[b_location] = objects[1]

    weightsLocationC = [0.25, 0.25, 0.25, 0.25]
    selectedWeight = weightsLocationC[a_location]+weightsLocationC[b_location]
    for i in range(4):
        weightsLocationC[i] += selectedWeight / 2.0
    weightsLocationC[a_location] = 0
    weightsLocationC[b_location] = 0
    c_location = np.random.choice(4, 1, p=weightsLocationC)[0]
    locations[c_location] = objects[2]

    weightsLocationD = [1.0, 1.0, 1.0, 1.0]
    weightsLocationD[a_location]=0
    weightsLocationD[b_location] = 0
    weightsLocationD[c_location] = 0
    d_location = np.random.choice(4, 1, p=weightsLocationD)[0]
    locations[d_location] = objects[3]
    res=[]
    for i in range(4):
        res.append(locations[i])
        state_toy_locations[locations[i]]=i
        state_locations_toy[i] = locations[i] # access toy type by location
    return tuple(res)


def run_env_function(cmd, parameter=None, parameter2=None):
    if cmd == "init_env":
        obj0, obj1, obj2, pbj3 = calc_locations()
        calc_rewards()
        shell_cmd = 'rosrun task4_env environment_functions.py {} {} {} {} {}'.format(cmd, obj0, obj1, obj2, pbj3)
        print('init environment with objects, running:"{}"'.format(shell_cmd))
        os.system(shell_cmd)
    if cmd == "pick":
        shell_cmd = 'rosrun task4_env environment_functions.py {} {}'.format(cmd, parameter)        
        print(shell_cmd)
        os.system(shell_cmd)
    if cmd == "place":
        shell_cmd = 'rosrun task4_env environment_functions.py {} {} {}'.format(cmd, parameter, parameter2)
        print(shell_cmd)
        os.system(shell_cmd)


def add_action(skill_name, parameters, observation, reward):
    actions.append((skill_name, parameters))
    observations.append(observation)
    rewards.append((reward))
    log_msg = "listed- action-{}, observation:{}, reward:{}".format((skill_name, parameters), observation, reward)
    log.append(log_msg)
    print(log_msg)
    total_reward_msg = "total rewards:{}".format(sum(rewards))
    print(total_reward_msg)


def callback_active():
    rospy.loginfo("Action server is processing the goal")


def callback_done(state, result):
    rospy.loginfo("Action server is done. State: %s, result: %s" % (str(state), str(result)))


def callback_feedback(feedback):
    rospy.loginfo("Feedback:%s" % str(feedback))


def handle_navigate(req): # Action 0 to 4. TODO -- students: Your agent should call this accordingly to its action choice (action 0-4, correspond to activating navigate with location 0 to 4)
    global pick_calls
    global nav_calls
    global state_robot_location
    original_location = state_robot_location
    nav_calls = nav_calls+1
    if pick_calls > 6 or nav_calls > 8:
        print("you used more than 6 picks, GAME OVER")
        add_action(skill_name="navigate", parameters="GAME-OVER", observation="GAME-OVER", reward=0)
        return navigateResponse(success=False);
        
    if len(actions) == 0:
        run_env_function("init_env")


    locations = [1.25, 0.85, 0.2], [2.25, 0.95, 0.2], [0.15516284108161926, 0.07012523710727692, 0.002471923828125], [3.0, -1.5, 0.2], [1.3, -0.5, 1]
    do_nav, obs_nav = calc_nav_succ(req.location)
    if do_nav:
	    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	    client.wait_for_server()
	    goal = MoveBaseGoal()
	    goal.target_pose.header.frame_id = "map"
	    goal.target_pose.header.stamp = rospy.Time.now()
	    goal.target_pose.pose.position.x = locations[req.location][0]  # req.goal.x
	    goal.target_pose.pose.position.y = locations[req.location][1]  # req.goal.y
	    goal.target_pose.pose.position.z = 0  # req.goal.z
	    goal.target_pose.pose.orientation.w = 1.0
	    client.send_goal(goal) 
	    wait = client.wait_for_result()
	    res = navigateResponse(success=False);

	    if not wait:
	        print("----------------not_wait----------------------")
	        rospy.logerr("Action server not available!")
	        rospy.signal_shutdown("Action server not available!")

	    else:
	        res.success = client.get_state() == SUCCEEDED

	    if state_robot_location == req.location:
	        res.success = False
	    state_robot_location = req.location
    print("state_robot_location({}) == req.location({})".format(state_robot_location , req.location))	    
    if original_location == req.location:
    	if do_nav != obs_nav:
    	    obs_nav=True
    	else:
    	    obs_nav=False
    	do_nav=False
    	print("navigate to self location: failed")
    if do_nav:
        _reward = -1
    else:
        _reward = -2
    print("success:{}, obs_noise:{},response:{}".format(do_nav,(do_nav != obs_nav), obs_nav))
    res = navigateResponse(success=obs_nav);
    add_action(skill_name="navigate", parameters=req, observation=res, reward=_reward)
    return res


def handle_info(req): # accessible by agent
    info = "This is how you get your state and reward info. Also how you determine which toy type to send to the pick request (the one at the robots location).\n"+ "\n"
    info = info + get_state() + "\n"+ "\n"
    info = info + "log:\n" + str(log) + "\n\n"
    info = info + "total rewards:{}".format(sum(rewards))
    return infoResponse(internal_info=info)


def handle_pick(req): # This is action 5. The agent only chooses the action pick, you need to make sure that the right argument gets passed. TODO Students -- your agents pick skill should: access info service, compare robot location to state_locations_toy (to know which object type to pass to the pick skill), execute a /pick service call with the correct object type. In case state_robot_location
    global state_robot_location
    global pick_calls
    global state_is_holding
    global state_holding_object_name
    global state_toy_locations
    global state_locations_toy
    pick_calls = pick_calls + 1    
    if pick_calls > 6 or nav_calls > 8:
        print("you used more than 6, GAME OVER")
        add_action(skill_name="pick", parameters="GAME-OVER", observation="GAME-OVER", reward=0)
        return pickResponse(success=False)
    if len(actions) == 0:
        run_env_function("init_env")        
    print('handle_pick')
    print(req)

    res = None
    reward = -2
    if state_is_holding or state_robot_location == 4:
        res = pickResponse(success=False)        
    
    elif req.toy_type not in state_toy_locations.keys(): # in case empty location
        print("Big F")
        res = pickResponse(success=False)
         
        
    elif state_robot_location == state_toy_locations[req.toy_type]:
        run_env_function("pick", req.toy_type) 
        reward = -1
        res = pickResponse(success=True)
        state_holding_object_name = req.toy_type
        state_is_holding=True
        state_toy_locations[req.toy_type] = 5 # toy in robot arm
        
        state_locations_toy[state_robot_location] = "None"
        state_locations_toy[5] = req.toy_type
    
    else:
        res = pickResponse(success=False)
    print('state_holding_object_name:{}'.format(state_holding_object_name))

    add_action(skill_name="pick", parameters=req, observation=res, reward=reward)
    return res



def handle_place(req):
    global state_holding_object_name
    global state_robot_location
    global state_toy_locations
    global state_locations_toy
    global state_toy_rewards
    global state_is_holding
    global pick_calls
    if pick_calls > 6 or nav_calls > 8:
        print("you used more than 6 picks, GAME OVER")
        add_action(skill_name="place", parameters="GAME-OVER", observation="GAME-OVER", reward=0)
        return placeResponse(success=False)
    print('state_holding_object_name:{}'.format(state_holding_object_name))
    print('state_robot_location:{}'.format(state_robot_location))
    if len(actions) == 0:
        run_env_function("init_env")
    print('handle_place')
    print(req)
    reward = 0
    res = None
    if state_holding_object_name == None:
        reward = -3
        res = placeResponse(success=False)
    elif state_robot_location !=4:
        reward = -1
        res = placeResponse(success=False)
    else:
        reward = state_toy_rewards[state_holding_object_name]
        state_toy_locations[state_holding_object_name]=state_robot_location
        run_env_function("place", state_holding_object_name, state_robot_location)
        state_holding_object_name=None
        state_locations_toy[5] = None
        state_is_holding = False
        res = placeResponse(success=True)


    add_action(skill_name="place", parameters=req, observation=res, reward=reward)
    return res
    
    

def load_servers(): 
    s = rospy.Service('/navigate', navigate, handle_navigate)
    print("Ready to navigate.")
    s = rospy.Service('/pick', pick, handle_pick)
    print("Ready to pick.")
    s = rospy.Service('/place', place, handle_place)
    print("Ready to place.")
    s = rospy.Service('/info', info, handle_info)
    rospy.spin()



if __name__ == '__main__':
    try:
        print("starting skills server node")
        rospy.init_node('skills_server_node')
        load_servers()
        # import pdb
        # pdb.set_trace()

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Skills server finished.")
