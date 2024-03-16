import pprint
import rospy
from task4_env.srv import *
import os
import roslaunch

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
    state_info = internal_info.split("\n\n")[1][:-1]
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
    total_rewards = int(internal_info.split("\n\n")[3][14:])

    return state, log, total_rewards


########################### CONTROL START ################################3
# toy_type options
GREEN = 'green'
BLUE = 'blue'
BLACK = 'black'
RED = 'red'
TOY_TYPES = [GREEN, BLUE, BLACK, RED]

LOCATION0_TOY_ORDER = [BLUE, BLACK, RED, GREEN]
LOCATION1_TOY_ORDER = [BLACK, RED, BLUE, GREEN]
LOCATION2_TOY_ORDER = [GREEN, BLACK, RED, BLUE]
LOCATION3_TOY_ORDER = [BLACK, RED, BLUE, GREEN]
LOCATIONS_TOY_ORDERS = [LOCATION0_TOY_ORDER, LOCATION1_TOY_ORDER, LOCATION2_TOY_ORDER, LOCATION3_TOY_ORDER]

navigations_left = 8
picks_left = 6
knapsack_toy = None



def call_navigate(location):
    global navigations_left
    try:
        navigate_srv = rospy.ServiceProxy('navigate', navigate)
        resp = navigate_srv(location)
        print(f"Navigating to {location}: {resp.success}")
        navigations_left -= 1
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def call_pick(toy_type):
    global knapsack_toy
    global picks_left
    try:
        pick_srv = rospy.ServiceProxy('pick', pick)
        resp = pick_srv(toy_type)
        if resp.success:
            knapsack_toy = toy_type
        print(f"Picking {toy_type}: {resp.success}")
        picks_left -= 1
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def call_place():
    global knapsack_toy
    try:
        place_srv = rospy.ServiceProxy('place', place)
        resp = place_srv()
        if resp.success:
            knapsack_toy = None
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


def place_toy_at_baby():
    if not call_navigate(4):
        if navigations_left > 0:
            call_navigate(4)
        else:
            return False
    
    return call_place()

def pick_toy(location, given_toys):
    toy_order = [toy for toy in LOCATIONS_TOY_ORDERS[location] if toy not in given_toys]
    for toy in toy_order:
        if picks_left < 1:
            return None
        if call_pick(toy):
            return toy
    
    return None

def get_and_pick_toy(location, given_toys):
    if navigations_left < 2:
        return None

    if not call_navigate(location):
        # check if there are enough navigations left for navigating again and to the baby
        if navigations_left >= 2 and not call_navigate(location) and navigations_left == 2:
            call_navigate(location)
        
    if navigations_left < 1:
        return None

    return pick_toy(location, given_toys)

def calc_next_location(left_locations):
    if len(left_locations) == 0:
        return None
    
    if len(left_locations) == 2: # assume locations doesn't include 0
        if navigations_left == 2:
            return 2
        else:
            return 1

    return left_locations[0]

def reset_control():
    global knapsack_toy
    global navigations_left
    global picks_left

    navigations_left = 8
    picks_left = 6
    knapsack_toy = None


def run_control():
    reset_control()

    given_toys = []
    next_locations = [2, 3, 1]
    next_location = next_locations[0]
    while next_location is not None: 
        if navigations_left < 2 or picks_left < 1:
            break
        toy = get_and_pick_toy(next_location, given_toys)
        if toy is not None:
            if place_toy_at_baby():
                given_toys.append(toy)
                next_locations.remove(next_location)
        
        next_location = calc_next_location(next_locations)

############################ CONTROL END ###############################3


############################ EXPERIMENT START ###############################3
# skills_server node global launcher
skills_server_process = None
skills_server_node = roslaunch.core.Node("task4_env", "skills_server.py", name="skills_server_node")#, output='screen')

launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

def relaunch_skills_server():
    # kill the existing skills_server
    global skills_server_process
    skills_server_process.stop() if skills_server_process else os.system("rosnode kill skills_server_node")
    rospy.sleep(1)
    print("### killed skills_server ###")
    
    # launch the skills_server
    skills_server_process = launch.launch(skills_server_node)
    rospy.sleep(1)

    # wait for services launched in server
    print("waiting for services...")
    rospy.wait_for_service('info')
    rospy.wait_for_service('navigate')
    rospy.wait_for_service('pick')
    rospy.wait_for_service('place')
    print("### started skills_server ###")

def reset_env():
    print(f"========== reset env =========")
    # make sure the navigation is running
    relaunch_skills_server()

    # start at the baby
    if not call_navigate(4):
        rospy.sleep(3)  

        # make sure it did not fail
        call_navigate(4)
        rospy.sleep(3)

    # spawn toys and reset counters
    relaunch_skills_server()  


################################
def print_infos(infos):
    print("\n\n===============================")
    print(f"==========INFO SUMMARY=========")
    print("===============================")
    for i, info in enumerate(infos):
        print(f"###\nControl {i+1}:\n###\n")
        print(info)
    print("===============================")

# todo validate file path
def export_infos(infos, average_reward):
    with open('experiment_output.txt', 'w') as f:
        for i, info in enumerate(infos):
            f.write(f"=== Info for expirament {i}:\n{info}\n\n")
        f.write(f"=== Average reward: {average_reward}\n\n\n")

def run_experiment(times=3):
    infos = []
    total_rewards = []
    for i in range(times):
        reset_env()

        print("\n\n===============================")
        print(f"========== CONTROL {i+1} =========")
        print("===============================")
        run_control()
        
        info = call_info()
        infos.append(info)
        total_reward = int(info.split("total rewards:")[1])
        total_rewards.append(total_reward)
        print(f"Total reward: {total_reward}")
        
        print(f"========== Finished {i+1}, Total reward: {total_reward} =========\n\n")
    
    print_infos(infos)
    average_reward = sum(total_rewards) / len(total_rewards)
    print(f"Average reward: {average_reward}")

    export_infos(infos, average_reward)


######################### EXPERIMENT END ##################################3


if __name__ == '__main__':
    relaunch_skills_server()
    internal_info = call_info()
    state, log, total_rewards = parse_info(internal_info)
    pprint.pprint(state)
    pprint.pprint(log)
    print(total_rewards)

    run_experiment()
