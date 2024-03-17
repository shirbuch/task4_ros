import pprint
from requests import get
import rospy
from task4_env.srv import *
import os
import roslaunch


### Globals ###
# toy_type options
GREEN = 'green'
BLUE = 'blue'
BLACK = 'black'
RED = 'red'

navigations_left = 8
picks_left = 6

NAVIGATE0 = 0
NAVIGATE1 = 1
NAVIGATE2 = 2
NAVIGATE3 = 3
NAVIGATE4 = 4
PICK = 5
PLACE = 6
ACTIONS = [NAVIGATE0, NAVIGATE1, NAVIGATE2, NAVIGATE3, NAVIGATE4, PICK, PLACE]

STATE_ROBOT_LOCATION_INDEX = 0
STATE_TOYS_LOCATION_INDEX = 1

BABY_LOCATION = 4
KNAPSACK_LOCATION = 5
POSSIBLE_ROBOT_LOCATIONS = [0, 1, 2, 3, BABY_LOCATION]
POSSIBLE_TOYS_LOCATIONS = [0, 1, 2, 3, BABY_LOCATION, KNAPSACK_LOCATION]


### State Class ###
class State:
    def __init__(self, state):
        self.robot_location = state[STATE_ROBOT_LOCATION_INDEX]
        toys_location = state[STATE_TOYS_LOCATION_INDEX]
        self.red_location = toys_location[RED]
        self.green_location = toys_location[GREEN]
        self.blue_location = toys_location[BLUE]
        self.black_location = toys_location[BLACK]

    def get_closeby_toy(self):
        if self.robot_location == BABY_LOCATION:
            return None
        elif self.red_location == self.robot_location:
            return RED
        elif self.green_location == self.robot_location:
            return GREEN
        elif self.blue_location == self.robot_location:
            return BLUE
        elif self.black_location == self.robot_location:
            return BLACK
        else:
            return None

    def get_holded_toy(self):
        if self.red_location == KNAPSACK_LOCATION:
            return RED
        elif self.green_location == KNAPSACK_LOCATION:
            return GREEN
        elif self.blue_location == KNAPSACK_LOCATION:
            return BLUE
        elif self.black_location == KNAPSACK_LOCATION:
            return BLACK
        else:
            return None

    def __str__(self):
        return f"Robot: {self.robot_location}\nRed: {self.red_location}, Green: {self.green_location}, Blue: {self.blue_location}, Black: {self.black_location}"   


### skills_server node global launcher ###
skills_server_process = None
skills_server_node = roslaunch.core.Node("task4_env", "skills_server.py", name="skills_server_node", output='log')

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()


### internal_info parser ###
def parse_dict(str, key_type="str"):
    parsed_dict = {}
    for keyval in str:
        if key_type == "str":
            key = keyval.split(":")[0].strip("'")
            val = int(keyval.split(":")[1])
        else:
            key = int(keyval.split(":")[0])
            val = keyval.split(":")[1].strip("'")
        parsed_dict[key] = val
    
    return parsed_dict

def parse_state(state_info):
    state_info = state_info.replace("\\", "")
    for i in range(4):
        state_info = state_info.replace(": ", ":")
    for i in range(4):
        state_info = state_info.replace(", ", ",")

    vars = [var for var in state_info.split(" ")[1:] if var != ""]

    outs = []
    for var in vars:
        var = var.split(":", 1)
        if var[1].startswith("{"):
            dict_pairs = var[1][1:-1].split(",")
            for pair in dict_pairs:
                pair = pair.split(":")
            outs.append(dict_pairs)
        else:
            outs.append(var[1])
        
    robot_location = int(outs[0])
    toys_location = parse_dict(outs[1])
    locations_toy = parse_dict(outs[2], key_type="int")
    toys_reward = parse_dict(outs[3])
    holding_toy = bool(outs[4])
    
    return [robot_location, toys_location, locations_toy, toys_reward, holding_toy]

def parse_info(internal_info):
    state = parse_state(internal_info.split("\n\n")[1][:-1])    
    log = internal_info.split("\n\n")[2][6:-1] # todo: parse log
    total_reward = int(internal_info.split("\n\n")[3][14:])

    return state, log, total_reward

def get_info():
    # Usage: state, log, total_reward = get_info()
    return parse_info(call_info())

def get_state() -> State:
    state, _, _ = get_info()
    return State(state)


### Service Calls ###
# Dumb, that is they do not check if the action is possible to execute but only wrapper for node call #
def call_navigate(location):
    global navigations_left
    try:
        print(f"Navigating to {location}: ", end =" ")
        navigate_srv = rospy.ServiceProxy('navigate', navigate)
        resp = navigate_srv(location)
        navigations_left -= 1
        print(resp.success)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def call_pick():
    global picks_left
    try:
        toy_type = get_state().get_closeby_toy()
        if toy_type is None:
            print("No toy to pick")
            return False

        pick_srv = rospy.ServiceProxy('pick', pick)
        resp = pick_srv(toy_type)
        print(f"Picking {toy_type}: {resp.success}")
        picks_left -= 1
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def call_place():
    try:
        place_srv = rospy.ServiceProxy('place', place)
        resp = place_srv()
        print(f"Placing: {resp.success}")
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def call_info():
    try:
        info_srv = rospy.ServiceProxy('info', info)
        resp = info_srv()
        return resp.internal_info
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


### RL Action Decision ###
# todo
def choose_action():
    # todo make table of all valid options
    # todo dont do bad stuff, just eliminate it...
    return PICK #!!!

def perform_action(action):
    if action == NAVIGATE0:
        call_navigate(0)
    elif action == NAVIGATE1:
        call_navigate(1)
    elif action == NAVIGATE2:
        call_navigate(2)
    elif action == NAVIGATE3:
        call_navigate(3)
    elif action == NAVIGATE4:
        call_navigate(4)
    elif action == PICK:
        call_pick()
    elif action == PLACE:
        call_place()


### Reset for New Runs ###
def relaunch_skills_server():
    # kill the existing skills_server
    global skills_server_process
    
    skills_server_process.stop() if skills_server_process else os.system("rosnode kill skills_server_node")
    print("### killed skills_server ###")
    
    # launch the skills_server
    skills_server_process = launch.launch(skills_server_node)

    # wait for services launched in server
    print("waiting for services...")
    rospy.wait_for_service('info')
    rospy.wait_for_service('navigate')
    rospy.wait_for_service('pick')
    rospy.wait_for_service('place')
    print("### started skills_server ###")

def reset_env():
    global navigations_left
    global picks_left
    navigations_left = 8
    picks_left = 6
    
    # Assumes skills_server is already running
    print(f"========== reset env =========")

    # start at the baby
    if not call_navigate(4):
        # make sure it did not fail
        call_navigate(4)

    # spawn toys and reset counters
    relaunch_skills_server()


### Main Functions ###
def run_control():
    action = choose_action()
    while action is not None:
        perform_action(action)
        action = choose_action()

def run_experiment(times=3):
    total_rewards = []
    for i in range(times):
        reset_env()

        print("\n\n===============================")
        print(f"========== CONTROL {i+1} =========")
        print("===============================")
        run_control()
        
        _, _, total_reward = get_info()
        total_rewards.append(total_reward)
        print(f"Total reward: {total_reward}")
        
        print(f"========== Finished {i+1}, Total reward: {total_reward} =========\n\n")
    
    average_reward = sum(total_rewards) / len(total_rewards)
    print(f"Average reward: {average_reward}")

def main():
    relaunch_skills_server()
    state, log, total_reward = get_info()
    pprint.pprint(state)
    # pprint.pprint(log)
    # print(total_reward)

    print(get_state())

    run_experiment()

# todo: get flag of learning or running
if __name__ == '__main__':
    try:
        main()
    finally:
        # After Ctrl+C, stop all nodes from running
        if skills_server_process:
            skills_server_process.stop()

