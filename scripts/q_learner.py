import datetime
import pprint
import random
import sys
from typing import Dict, List, Tuple
import rospy
import os
import roslaunch
from task4_env.srv import *
import pickle
import random
from geometry_msgs.msg import Point
from task4_env.srv import navigate, navigateResponse, pick, pickResponse, place, placeResponse, info, infoResponse
import numpy as np

# Important! Run the code from the catkin/src folder, or update this path
Q_TABLE_FILE_PATH = "task4_env/most_recent_q_table.pkl"

VERBOSE = False


### Constants ###
class Locations:
    BABY_LOCATION = 4
    KNAPSACK_LOCATION = 5
    ROBOT_LOCATIONS = [0, 1, 2, 3, BABY_LOCATION]
    INITIAL_TOYS_LOCATIONS = [0, 1, 2, 3]
    TOYS_LOCATIONS = INITIAL_TOYS_LOCATIONS + [BABY_LOCATION, KNAPSACK_LOCATION]

class ToyTypes:
    RED = 'red'
    GREEN = 'green'
    BLUE = 'blue'
    BLACK = 'black'


### Classes ###
class Action:
    NAVIGATE0 = 0
    NAVIGATE1 = 1
    NAVIGATE2 = 2
    NAVIGATE3 = 3
    NAVIGATE4 = 4
    PICK = 5
    PLACE = 6
    NAVIGATION_ACTIONS = [NAVIGATE0, NAVIGATE1, NAVIGATE2, NAVIGATE3, NAVIGATE4]
    ALL_ACTIONS = NAVIGATION_ACTIONS + [PICK, PLACE]

    def __init__(self, action_id):
        self.action_id = action_id

    def __str__(self):
        if self.action_id == Action.NAVIGATE0:
            return "Navigate 0"
        elif self.action_id == Action.NAVIGATE1:
            return "Navigate 1"
        elif self.action_id == Action.NAVIGATE2:
            return "Navigate 2"
        elif self.action_id == Action.NAVIGATE3:
            return "Navigate 3"
        elif self.action_id == Action.NAVIGATE4:
            return "Navigate 4"
        elif self.action_id == Action.PICK:
            return "Pick      "
        elif self.action_id == Action.PLACE:
            return "Place     "
        else:
            return "None"

    def key(self):
        return self.action_id
    
    def __hash__(self):
        return hash(self.key())  
    
    def __eq__(self, other):
        return self.key() == other.__hash__()
    
    def copy(self):
        return Action(self.action_id)
    
    def is_navigation(self):
        return self.action_id in Action.NAVIGATION_ACTIONS
    
    def is_pick(self):
        return self.action_id == Action.PICK
    
    def is_place(self):
        return self.action_id == Action.PLACE

    def get_location_of_navigation_action(self):
        if self.action_id == Action.NAVIGATE0:
            return 0
        elif self.action_id == Action.NAVIGATE1:
            return 1
        elif self.action_id == Action.NAVIGATE2:
            return 2
        elif self.action_id == Action.NAVIGATE3:
            return 3
        elif self.action_id == Action.NAVIGATE4:
            return 4
        else:
            return None
    
    def perform(self, skills_server) -> bool:
        if self.is_navigation():
            return skills_server.navigate(self.get_location_of_navigation_action())
        elif self.is_pick():
            toy_type = skills_server.get_info().state.get_closeby_toy()
            if toy_type is None:
                print("No toy to pick")
                return False

            return skills_server.pick(toy_type)
        elif self.is_place():
            return skills_server.place()
    
class State:
    MAX_NAVIGATIONS = 8
    MAX_PICKS = 6

    @staticmethod
    def toy_locations_to_dict(red_location, green_location, blue_location, black_location):
        return {ToyTypes.RED: red_location, ToyTypes.GREEN: green_location, ToyTypes.BLUE: blue_location, ToyTypes.BLACK: black_location}

    def __init__(self, robot_location, toys_location, navigations_left=MAX_NAVIGATIONS, picks_left=MAX_PICKS):
        self.robot_location = robot_location
        self.toys_location = toys_location
        self.navigations_left = navigations_left
        self.picks_left = picks_left
      
    def __str__(self):
        return f"(Robot: {self.robot_location}, red: {self.toys_location[ToyTypes.RED]}, green: {self.toys_location[ToyTypes.GREEN]}, blue: {self.toys_location[ToyTypes.BLUE]}, black: {self.toys_location[ToyTypes.BLACK]}, navigations left: {self.navigations_left}, picks left: {self.picks_left})"

    def __eq__(self, other):
        return self.key() == other.key()

    def copy(self):
        return State(self.robot_location, self.toys_location.copy(), self.navigations_left, self.picks_left)

    def key(self):
        return (self.robot_location, tuple(self.toys_location.values()), self.navigations_left, self.picks_left)
    
    def __hash__(self):
        return hash(self.key())
    
    def get_closeby_toy(self):
        if self.robot_location == Locations.BABY_LOCATION:
            return None
        elif self.toys_location[ToyTypes.RED] == self.robot_location:
            return ToyTypes.RED
        elif self.toys_location[ToyTypes.GREEN] == self.robot_location:
            return ToyTypes.GREEN
        elif self.toys_location[ToyTypes.BLUE] == self.robot_location:
            return ToyTypes.BLUE
        elif self.toys_location[ToyTypes.BLACK] == self.robot_location:
            return ToyTypes.BLACK
        else:
            return None

    def get_holded_toy(self):
        if self.toys_location[ToyTypes.RED] == Locations.KNAPSACK_LOCATION:
            return ToyTypes.RED
        elif self.toys_location[ToyTypes.GREEN] == Locations.KNAPSACK_LOCATION:
            return ToyTypes.GREEN
        elif self.toys_location[ToyTypes.BLUE] == Locations.KNAPSACK_LOCATION:
            return ToyTypes.BLUE
        elif self.toys_location[ToyTypes.BLACK] == Locations.KNAPSACK_LOCATION:
            return ToyTypes.BLACK
        else:
            return None

    def is_holding_toy(self):
        return self.get_holded_toy() is not None
    
    def get_toys_location_list(self):
        return list(self.toys_location.values())    
  
    def get_used_picks(self):
        return State.MAX_PICKS - self.picks_left

    def get_used_navigations(self):
        return State.MAX_NAVIGATIONS - self.navigations_left
    
    def is_valid(self):
        toy_locations_list = self.get_toys_location_list()

        ## Checks by number of moved toys
        number_of_moved_toys = sum(1 for toy_location in toy_locations_list if toy_location not in Locations.INITIAL_TOYS_LOCATIONS)

        # Check if the number of moved toys is not the same as the number of used picks (pick always succeeds if there is a toy nearby)
        if number_of_moved_toys != self.get_used_picks():
            return False
        
        # Check if the number of moved toys is not possible with the number of used navigations
        if self.get_used_navigations() < number_of_moved_toys*2 - 1:
            return False
        
        # Check if the robot is holding more than one toy
        if toy_locations_list.count(Locations.KNAPSACK_LOCATION) > 1:
            return False

        # Check if there are two toys in the same start location
        toy_locations_in_inital_location = [toy_location for toy_location in toy_locations_list if toy_location in Locations.INITIAL_TOYS_LOCATIONS]
        if len(set(toy_locations_in_inital_location)) != len(toy_locations_in_inital_location):
            return False

        # Check if the robot is at a toy and has all navigations left
        if self.robot_location != Locations.BABY_LOCATION and self.navigations_left == State.MAX_NAVIGATIONS:
            return False
        
        return True

class StateAction:
    def __init__(self, state: State, action: Action):
        self.state = state
        self.action = action

    def __eq__(self, other):
        return self.__hash__() == other.__hash__()
    
    def key(self):
        return (self.state.key(), self.action.key())
    
    def __hash__(self):
        return hash(self.key())

    def __str__(self):
        return f"{self.state}, {self.action}"
    
    def copy(self):
        return StateAction(self.state.copy(), self.action.copy())

    def is_reasonable(self):
        # NAVIGATE
        if self.action.is_navigation():
            ## Checks about navigations and picks left
            # Check if there are no more navigations left
            if self.state.navigations_left < 1:
                return False

            # Check if the robot is at the baby and doesn't have enough navigations/picks to get give the baby another toy (reqires at least 2 navigations and 1 pick)
            if self.state.robot_location == Locations.BABY_LOCATION and (self.state.navigations_left < 2 or self.state.picks_left < 1):
                return False

            ## Checks about the navigation target
            navigation_target = self.action.get_location_of_navigation_action()

            # Check if navigating to the baby (without holding a toy)
            if navigation_target == Locations.BABY_LOCATION and not self.state.is_holding_toy():
                return False
            
            # Check if navigating to the same location
            if navigation_target == self.state.robot_location:
                return False

            # Check if navigating to an initial toy location with no toy there
            if navigation_target in Locations.INITIAL_TOYS_LOCATIONS \
                and navigation_target not in self.state.get_toys_location_list():
                return False
            
            # Check if navigating from one toy to another (doens't make sence to move between toys when Pick will always succeed if there is a toy nearby)
            if self.state.robot_location in Locations.INITIAL_TOYS_LOCATIONS and navigation_target in Locations.INITIAL_TOYS_LOCATIONS:
                return False

            # When holding a toy, only allow navigating to the baby (as for navigation action)
            if self.state.is_holding_toy() and navigation_target != Locations.BABY_LOCATION:
                return False
        
        # Actions.PICK
        elif self.action.is_pick():
            # Check if there are no more picks left
            if self.state.picks_left < 1:
                return False
            
            # Check if there are no navigations left to deliver the picked toy
            if self.state.navigations_left < 1:
                return False

            # Check if the robot is at the baby # backlog: remove duplicate from get_closeby_toy down below
            if self.state.robot_location == Locations.BABY_LOCATION:
                return False

            # Check if the robot is holding a toy
            if self.state.is_holding_toy():
                return False

            # Check if there is no toy nearby to pick
            if self.state.get_closeby_toy() is None:
                return False

        # Actions.PLACE
        elif self.action.is_place():
            # Check if the robot is not at the baby and holding a toy
            if self.state.robot_location != Locations.BABY_LOCATION or not self.state.is_holding_toy():
                return False
        
        return True


### Skills Server ###
# internal_info parser #
class InfoParser:
    @staticmethod
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

    @staticmethod
    def parse_state(state_info) -> Tuple[int, dict, dict]:
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
        toys_location = InfoParser.parse_dict(outs[1])
        # 
        # unuseful: locations_toy = InfoParser.parse_dict(outs[2], key_type="int")
        toys_reward = InfoParser.parse_dict(outs[3])
        # unuseful: holding_toy = bool(outs[4])

        return robot_location, toys_location, toys_reward
    
    @staticmethod
    def parse_info(internal_info) -> Tuple[int, dict, int, str, int]:
        robot_location, toys_location, toys_reward = InfoParser.parse_state(internal_info.split("\n\n")[1][:-1])    
        log = internal_info.split("\n\n")[2][6:-1] # backlog: parse log
        total_reward = int(internal_info.split("\n\n")[3][14:])

        return robot_location, toys_location, toys_reward, log, total_reward

class Info:
    def __init__(self, state, toys_reward, log, total_reward):
        self.state = state
        self.toys_reward = toys_reward
        self.log = log
        self.total_reward = total_reward

    def __str__(self):
        return f"State: {self.state}, Toys reward: {self.toys_reward}, Log: {self.log}, Total reward: {self.total_reward}"

### Server Simulator ###
class ServerSimulator:
    def __init__(self):
        self.SUCCEEDED = 3
        self.actions = []
        self.observations = []
        self.rewards = []
        self.log = []
        self.total_reward_msg = ""
        self.state_toy_locations={"green":0, "blue":1, "black":2, "red":3}
        self.state_locations_toy={0:"green", 1:"blue", 2:"black", 3:"red", 5:"None"}
        self.state_toy_rewards={"green":15, "blue":20, "black":30, "red":40}
        self.state_robot_location = 4
        self.state_is_holding= False
        self.state_holding_object_name = None
        self.objects=["green", "blue", "black", "red"]
        self.pick_calls=0
        self.nav_calls=0
        
    def reset(self):
        self.SUCCEEDED = 3
        self.actions = []
        self.observations = []
        self.rewards = []
        self.log = []
        self.total_reward_msg = ""
        self.state_toy_locations={"green":0, "blue":1, "black":2, "red":3}
        self.state_locations_toy={0:"green", 1:"blue", 2:"black", 3:"red", 5:"None"}
        self.state_toy_rewards={"green":15, "blue":20, "black":30, "red":40}
        self.state_robot_location = 4
        self.state_is_holding= False
        self.state_holding_object_name = None
        self.objects=["green", "blue", "black", "red"]
        self.pick_calls=0
        self.nav_calls=0

    # Helper Methods #
    def get_state(self):
        return "state:[robot location:{} toys_location:{} locations_toy:{} toys_reward:{} holding_toy:{}]".format(self.state_robot_location, self.state_toy_locations, self.state_locations_toy, self.state_toy_rewards, self.state_is_holding)

    def calc_rewards(self): # fixed reward per toy
        rewards=[15, 20, 30, 40]    
        a_reward = 0
        self.state_toy_rewards[self.objects[0]] = rewards[a_reward]
        b_reward = 1
        self.state_toy_rewards[self.objects[1]] = rewards[b_reward]
        c_reward = 2
        self.state_toy_rewards[self.objects[2]] = rewards[c_reward]
        d_reward = 3
        self.state_toy_rewards[self.objects[3]] = rewards[d_reward]
        # print("rewards:")
        # print(self.state_toy_rewards)

    def calc_nav_succ(self, location): # navigate fails, the no motion is executed in gazebo (state does not change, nav counter +1)
        weightsSuccess = [0.8,0.85,0.9,0.75,1.0] # navigation success is dependent on the location
        weightsObs = [1.0,1.0,1.0,1.0,1.0] # nav success is always observed correctly	
        succ = random.random() <= weightsSuccess[location]
        obs = random.random() <= weightsObs[location]
        # print("sampled navigate real obs:{}".format(obs))
        # print("sampled navigate succ:{}".format(succ))
        if obs:
            return (succ,succ)
        else:
            return (succ, not succ)

    def calc_locations(self): # we can leave this, the toys get distributed to location according to the same backstory
        locations = {}
        weightsLocationA = [0.1,0.05,0.8,0.05]
        a_location = np.random.choice(4, 1, p=weightsLocationA)[0]
        locations[a_location]=self.objects[0]

        weightsLocationB = [0.7,0.1,0.1,0.1]
        selectedWeight = weightsLocationB[a_location]
        for i in range(4):
            weightsLocationB[i] += selectedWeight/3.0
        weightsLocationB[a_location]=0
        b_location = np.random.choice(4, 1, p=weightsLocationB)[0]
        locations[b_location] = self.objects[1]

        weightsLocationC = [0.25, 0.25, 0.25, 0.25]
        selectedWeight = weightsLocationC[a_location]+weightsLocationC[b_location]
        for i in range(4):
            weightsLocationC[i] += selectedWeight / 2.0
        weightsLocationC[a_location] = 0
        weightsLocationC[b_location] = 0
        c_location = np.random.choice(4, 1, p=weightsLocationC)[0]
        locations[c_location] = self.objects[2]

        weightsLocationD = [1.0, 1.0, 1.0, 1.0]
        weightsLocationD[a_location]=0
        weightsLocationD[b_location] = 0
        weightsLocationD[c_location] = 0
        d_location = np.random.choice(4, 1, p=weightsLocationD)[0]
        locations[d_location] = self.objects[3]
        res=[]
        for i in range(4):
            res.append(locations[i])
            self.state_toy_locations[locations[i]]=i
            self.state_locations_toy[i] = locations[i] # access toy type by location
        return tuple(res)

    def run_env_function(self, cmd, parameter=None, parameter2=None):
        if cmd == "init_env":
            obj0, obj1, obj2, pbj3 = self.calc_locations()
            self.calc_rewards()
            shell_cmd = 'rosrun task4_env environment_functions.py {} {} {} {} {}'.format(cmd, obj0, obj1, obj2, pbj3)
            # print('init environment with objects, running:"{}"'.format(shell_cmd))
            # os.system(shell_cmd)
        if cmd == "pick":
            shell_cmd = 'rosrun task4_env environment_functions.py {} {}'.format(cmd, parameter)        
            # print(shell_cmd)
            # os.system(shell_cmd)
        if cmd == "place":
            shell_cmd = 'rosrun task4_env environment_functions.py {} {} {}'.format(cmd, parameter, parameter2)
            # print(shell_cmd)
            # os.system(shell_cmd)

    def add_action(self, skill_name, parameters, observation, reward):
        self.actions.append((skill_name, parameters))
        self.observations.append(observation)
        self.rewards.append((reward))
        log_msg = "listed- action-{}, observation:{}, reward:{}".format((skill_name, parameters), observation, reward)
        self.log.append(log_msg)
        # print(log_msg)
        # self.total_reward_msg = "total rewards:{}".format(sum(self.rewards))
        # print(self.total_reward_msg)

    # Service Handlers #
    def handle_navigate(self, req):
        original_location = self.state_robot_location
        self.nav_calls = self.nav_calls+1
        if self.pick_calls > 6 or self.nav_calls > 8:
            # print("you used more than 6 picks, GAME OVER")
            self.add_action(skill_name="navigate", parameters="GAME-OVER", observation="GAME-OVER", reward=0)
            return navigateResponse(success=False);
            
        if len(self.actions) == 0:
            self.run_env_function("init_env")

        locations = [1.25, 0.85, 0.2], [2.25, 0.95, 0.2], [0.15516284108161926, 0.07012523710727692, 0.002471923828125], [3.0, -1.5, 0.2], [1.3, -0.5, 1]
        do_nav, obs_nav = self.calc_nav_succ(req.location)
        if do_nav:
            # client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            # client.wait_for_server()
            # goal = MoveBaseGoal()
            # goal.target_pose.header.frame_id = "map"
            # goal.target_pose.header.stamp = rospy.Time.now()
            # goal.target_pose.pose.position.x = locations[req.location][0]  # req.goal.x
            # goal.target_pose.pose.position.y = locations[req.location][1]  # req.goal.y
            # goal.target_pose.pose.position.z = 0  # req.goal.z
            # goal.target_pose.pose.orientation.w = 1.0
            # client.send_goal(goal) 
            # wait = client.wait_for_result()
            res = navigateResponse(success=False)

            # if not wait:
            # 	print("----------------not_wait----------------------")
            # 	rospy.logerr("Action server not available!")
            # 	rospy.signal_shutdown("Action server not available!")
            # else:
                # res.success = client.get_state() == self.SUCCEEDED
            res.success = True #
   
            if self.state_robot_location == req.location:
                res.success = False
            self.state_robot_location = req.location
        # print("state_robot_location({}) == req.location({})".format(self.state_robot_location , req.location))        
        if original_location == req.location:
            if do_nav != obs_nav:
                obs_nav=True
            else:
                obs_nav=False
            do_nav=False
            # print("navigate to self location: failed")
        if do_nav:
            _reward = -1
        else:
            _reward = -2
        # print("success:{}, obs_noise:{},response:{}".format(do_nav,(do_nav != obs_nav), obs_nav))
        res = navigateResponse(success=obs_nav)
        self.add_action(skill_name="navigate", parameters=req, observation=res, reward=_reward)
        return res

    def handle_info(self): # accessible by agent
        info = "This is how you get your state and reward info. Also how you determine which toy type to send to the pick request (the one at the robots location).\n"+ "\n"
        info = info + self.get_state() + "\n"+ "\n"
        info = info + "log:\n" + str(self.log) + "\n\n"
        info = info + "total rewards:{}".format(sum(self.rewards))
        return infoResponse(internal_info=info)

    def handle_pick(self, req):
        self.pick_calls = self.pick_calls + 1    
        if self.pick_calls > 6 or self.nav_calls > 8:
            # print("you used more than 6, GAME OVER")
            self.add_action(skill_name="pick", parameters="GAME-OVER", observation="GAME-OVER", reward=0)
            return pickResponse(success=False)
        if len(self.actions) == 0:
            self.run_env_function("init_env")        
        # print('handle_pick')
        # print(req)

        res = None
        reward = -2
        if self.state_is_holding or self.state_robot_location == 4:
            res = pickResponse(success=False)        

        elif req.toy_type not in self.state_toy_locations.keys(): # in case empty location
            # print("Big F")
            res = pickResponse(success=False)
                
        elif self.state_robot_location == self.state_toy_locations[req.toy_type]:
            self.run_env_function("pick", req.toy_type) 
            reward = -1
            res = pickResponse(success=True)
            self.state_holding_object_name = req.toy_type
            self.state_is_holding=True
            self.state_toy_locations[req.toy_type] = 5 # toy in robot arm
            
            self.state_locations_toy[self.state_robot_location] = "None"
            self.state_locations_toy[5] = req.toy_type

        else:
            res = pickResponse(success=False)
        # print('state_holding_object_name:{}'.format(self.state_holding_object_name))

        self.add_action(skill_name="pick", parameters=req, observation=res, reward=reward)
        return res

    def handle_place(self, req=None):
        if self.pick_calls > 6 or self.nav_calls > 8:
            # print("you used more than 6 picks, GAME OVER")
            self.add_action(skill_name="place", parameters="GAME-OVER", observation="GAME-OVER", reward=0)
            return placeResponse(success=False)
        # print('state_holding_object_name:{}'.format(self.state_holding_object_name))
        # print('state_robot_location:{}'.format(self.state_robot_location))
        if len(self.actions) == 0:
            self.run_env_function("init_env")
        # print('handle_place')
        reward = 0
        res = None
        if self.state_holding_object_name == None:
            reward = -3
            res = placeResponse(success=False)
        elif self.state_robot_location !=4:
            reward = -1
            res = placeResponse(success=False)
        else:
            reward = self.state_toy_rewards[self.state_holding_object_name]
            self.state_toy_locations[self.state_holding_object_name]=self.state_robot_location
            self.run_env_function("place", self.state_holding_object_name, self.state_robot_location)
            self.state_holding_object_name=None
            self.state_locations_toy[5] = None
            self.state_is_holding = False
            res = placeResponse(success=True)

        self.add_action(skill_name="place", parameters=req, observation=res, reward=reward)
        return res

class GeneralReq:
    pass

# Skils Server Wrapper #
class SkillsServer:
    MAX_NAVIGATIONS = State.MAX_NAVIGATIONS
    MAX_PICKS = State.MAX_PICKS
    
    def __init__(self, verbose=False):
        self.navigations_left = SkillsServer.MAX_NAVIGATIONS
        self.picks_left = SkillsServer.MAX_PICKS
        self.verbose = verbose

        self.node = roslaunch.core.Node("task4_env", "skills_server.py", name="skills_server_node", output='log')
        self.launcher = roslaunch.scriptapi.ROSLaunch()
        self.launcher.start()
        self.process = None
        
        # For server simulation usage
        # self.server_simulator = ServerSimulator()
            
    def reset_parameters(self):
        self.navigations_left = SkillsServer.MAX_NAVIGATIONS
        self.picks_left = SkillsServer.MAX_PICKS
        
        # For server simulation usage
        # self.server_simulator.reset()
    
    # Operation #
    def kill(self):
        # For server simulation usage
        self.process.stop() if self.process else os.system("rosnode kill skills_server_node")
        
        self.process = None
    
    def launch(self):
        self.reset_parameters()
        
        # For server simulation usage
        # Comment these out
        self.launcher.launch(self.node)
        # wait for services launched in server
        rospy.wait_for_service('info')
        rospy.wait_for_service('navigate')
        rospy.wait_for_service('pick')
        rospy.wait_for_service('place')

    def relaunch(self):
        self.kill()
        self.launch()

    # Skills #
    def navigate(self, location) -> bool:
        self.navigations_left -= 1
        success = self.call_navigate(location)
        
        if self.verbose:
            print(f"Navigating to {location}: {success}")
        
        return success

    def pick(self, toy_type) -> bool:
        self.picks_left -= 1
        success = self.call_pick(toy_type)
        
        if self.verbose:
            print(f"Picking {toy_type}: {success}")
        
        return success
    
    def place(self) -> bool:
        success = self.call_place()
        
        if self.verbose:
            print(f"Placing: {success}")
        
        return success
    
    def get_info(self) -> Info: 
        internal_info = self.call_info()
        robot_location, toys_location, toys_reward, log, total_reward = InfoParser.parse_info(internal_info)
        state = State(robot_location, toys_location, self.navigations_left, self.picks_left)
        
        return Info(state, toys_reward, log, total_reward)

    # Service Calls #
    def call_navigate(self, location) -> bool:
        try:
            navigate_srv = rospy.ServiceProxy('navigate', navigate)
            resp = navigate_srv(location)
            
            # For server simulation usage
            # req = GeneralReq()
            # req.location = location
            # resp = self.server_simulator.handle_navigate(req)
            
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def call_pick(self, toy_type) -> bool:
        try:
            pick_srv = rospy.ServiceProxy('pick', pick)
            resp = pick_srv(toy_type)

            # For server simulation usage
            # req = GeneralReq()
            # req.toy_type = toy_type
            # resp = self.server_simulator.handle_pick(req)            
            
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def call_place(self) -> bool:
        try:
            place_srv = rospy.ServiceProxy('place', place)
            resp = place_srv()

            # For server simulation usage
            # resp = self.server_simulator.handle_place()  

            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def call_info(self) -> str:
        try:
            info_srv = rospy.ServiceProxy('info', info)
            resp = info_srv()

            # For server simulation usage
            # resp = self.server_simulator.handle_info()  

            return resp.internal_info
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


### Q-Learning ###
class QTable:
    q_table: Dict[StateAction, int]

    INIT_VALUE = 0
    
    eps = 0.1
    alpha = 0.01
    GAMMA = 0.95

    def __init__(self, file_path=None):
        if file_path is None or not os.path.exists(file_path):
            self.create_initial_q_table()
        else:
            with open(file_path, 'rb') as f:
                self.q_table = pickle.load(f)
        
    @staticmethod
    def print_q_table_formated_dict(q_table_formated_dict: dict, skip_printing_factor=1, what_to_print="all"):
        if not q_table_formated_dict:
            print("None\n")
            return
        
        if skip_printing_factor < 1:
            skip_printing_factor = 1
        i = 0
        for state_action, reward in q_table_formated_dict.items():
            if i % skip_printing_factor == 0:
                if what_to_print == "all":
                    print(f"{state_action.state}, {state_action.action}, Reward: {reward}")
                elif what_to_print == "state":
                    print(f"{state_action.state}")
                elif what_to_print == "action_reward":
                    print(f"{state_action.action}, Reward: {reward}")
            i += 1
    
    def get_reward(self, state: State, action: Action):
        return self.q_table[StateAction(state, action)]
    
    def print(self, skip_printing_factor=1000):
        QTable.print_q_table_formated_dict(self.q_table, skip_printing_factor)

    def print_state_records(self, state: State):
        QTable.print_q_table_formated_dict(self.get_state_records(state), what_to_print="all")
    
    def create_initial_q_table(self):
        # Q:SxA --> R
        # S: state, A: action, R: reward
        self.q_table = {}
        for robot_location in Locations.ROBOT_LOCATIONS:
            for red_location in Locations.TOYS_LOCATIONS:
                for green_location in Locations.TOYS_LOCATIONS:
                    for blue_location in Locations.TOYS_LOCATIONS:
                        for black_location in Locations.TOYS_LOCATIONS:
                            for navigations_left in range(State.MAX_NAVIGATIONS+1):
                                for picks_left in range(State.MAX_PICKS+1):
                                    state = State(robot_location, State.toy_locations_to_dict(red_location, green_location, blue_location, black_location), navigations_left, picks_left)
                                    if state.is_valid():
                                        for action in [Action(action_num) for action_num in Action.ALL_ACTIONS]:
                                            state_action = StateAction(state, action)
                                            if state_action.is_reasonable():
                                                self.update(state, action, QTable.INIT_VALUE)
    
    def get_state_records(self, state: State) -> Dict[StateAction, int]:
        state_records = {}
        for state_action, reward in self.q_table.items():
            if state_action.state == state:
                state_records[state_action] = reward
        return state_records
    
    def get_updated_records(self) -> Dict[StateAction, int]:
        updated_records = {}
        for state_action, reward in self.q_table.items():
            if reward != QTable.INIT_VALUE:
                updated_records[state_action] = reward
        return updated_records
    
    def get_num_updated_records(self) -> int:
        return len(self.get_updated_records())
    
    @staticmethod
    def generate_file_name():
        return "q_table_" + datetime.datetime.now().strftime("%d-%m_%H-%M") + ".pkl"
    
    def export(self, file_path):
        with open(file_path, 'wb') as f:
            pickle.dump(self.q_table, f)
        
        return file_path

    def update(self, state: State, action: Action, reward: int):
        self.q_table[StateAction(state, action)] = reward

    def fit(self, state: State, action: Action, action_reward: int, next_state: State):
        current_state_action_reward = self.get_reward(state, action)
        next_state_records = self.get_state_records(next_state)
        if not next_state_records:
            max_next_state_action_reward = 0
        else:
            max_next_state_action = QTable.get_max_record_from_q_table_formated_dict(next_state_records)
            max_next_state_action_reward = max_next_state_action[1]
        
        reward = (1 - QTable.alpha)*current_state_action_reward + QTable.alpha*(action_reward + QTable.GAMMA*max_next_state_action_reward)
        self.update(state, action, reward)
    
    @staticmethod
    def get_max_record_from_q_table_formated_dict(state_records: dict) -> tuple:
        return max(state_records.items(), key=lambda item: item[1])
    
    def get_max_record(self) -> tuple:
        return QTable.get_max_record_from_q_table_formated_dict(self.q_table)

    def choose_next_action(self, state: State, learning_mode=False, verbose=True) -> Action:
        # Eps-Greedy policy
        state_records = self.get_state_records(state)
        if not state_records:
            return None

        if verbose:
            print(f"\nState: {state}\nOptions:")
            QTable.print_q_table_formated_dict(state_records, what_to_print="action_reward")
        
        if learning_mode and random.random() < QTable.eps:
            state_action, reward = random.choice(list(state_records.items()))
        else:
            state_action = QTable.get_max_record_from_q_table_formated_dict(state_records)[0]
        return state_action.action

    def checkup(self):
        # Check that all the states are valid, and that all their corresponding actions are reasonable
        for state_action, reward in self.q_table.items():
            if not state_action.state.is_valid():
                print(f"Warning: Invalid State in q_table, first found:\n{state_action.state}")
                return
            if not state_action.is_reasonable():
                print(f"Warning: Not reasonable record in q_table, first found:\n{state_action}")
                return

        # Check duplicates of keys
        updated_records = self.get_updated_records().items()
        for state_action, reward in updated_records:
            num_duplicated = sum(1 for not_updated_state_action, not_updated_reward in self.q_table.items() - updated_records if state_action == not_updated_state_action)
            if num_duplicated > 0:
                print(f"Warning: Duplicate key in q_table {num_duplicated} times! for:\n{state_action},")
                for state_record in self.get_state_records(state_action.state).items():
                    if state_record[0].action == state_action.action:
                        print(f"{state_record[0].__str__()}, {state_record[0].__hash__()}")

                return


### Experiment Runner ###
class ExperimentRunner:
    DEFAULT_LEARNING_ITERATIONS = 10
    DEFAULT_EXPORT_RATE = 1
    EXPERIMENT_ITERATIONS = 10
    
    def __init__(self, skills_server: SkillsServer, import_file_path=None, verbose=None, learning_mode=False, iterations=None, export_file_path=None, export_rate=None):
        self.skills_server = skills_server
        if import_file_path is None or not os.path.exists(import_file_path):
            raise Exception(f"Q-table file does not exist at: {import_file_path}, please update the path in Q_TABLE_FILE_PATH constant in q_learner.py (or make sure to run from the relative dir to the file)")
        self.q_table = QTable(import_file_path)
        self.verbose = not learning_mode if verbose is None else verbose
        
        self.learning_mode = learning_mode
        if learning_mode:
            self.iterations = iterations if iterations is not None else ExperimentRunner.DEFAULT_LEARNING_ITERATIONS
            self.export_file_path = export_file_path if export_file_path is not None else Q_TABLE_FILE_PATH # backlog: check file exists
            self.export_rate = export_rate if export_rate is not None else ExperimentRunner.DEFAULT_EXPORT_RATE
        else:
            self.iterations = iterations if iterations is not None else ExperimentRunner.EXPERIMENT_ITERATIONS
        
    def reset_env(self):
        # Assumes skills_server is already running
        if self.verbose:
            print(f"========== reset env =========")

        # Spawn toys and start at the baby
        if not self.skills_server.navigate(4):
            # make sure it did not fail
            self.skills_server.navigate(4)

        # Reset counters
        self.skills_server.relaunch()

    def get_state_and_next_action(self):
        state = self.skills_server.get_info().state      
        return state, self.q_table.choose_next_action(state, self.learning_mode, self.verbose)
    
    def run_control(self): 
        state, action = self.get_state_and_next_action()
        prev_total_reward = 0
        while action is not None:
            action.perform(self.skills_server)
            
            info = self.skills_server.get_info()
            next_state = info.state
            total_reward = info.total_reward
            action_reward = total_reward - prev_total_reward

            if self.learning_mode:
                self.q_table.fit(state, action, action_reward, next_state)
            
            if self.verbose:
                print(f"Performed Action: {action}")
                print(f"Action Reward: {action_reward}")
                print(f"Total reward: {total_reward}\n")
            
            prev_total_reward = total_reward
            state, action = self.get_state_and_next_action()

    def run_experiment(self):
        self.skills_server.relaunch()

        if self.learning_mode:
            prev_updated_records = self.q_table.get_num_updated_records()
    
        total_rewards = []
        for i in range(self.iterations):
            self.reset_env()

            print(f"\n\n========== ITERATION {i+1} =========")
            if self.verbose:
                info = self.skills_server.get_info()
                print(f"Initial state: {info.state}")
                print(f"Toys rewards: {info.toys_reward}")

            self.run_control()
            
            info = self.skills_server.get_info()
            total_reward = info.total_reward
            total_rewards.append(total_reward)
                
            print(f"========== Finished {i+1}, Total reward: {total_reward} =========")
            
            if self.learning_mode and i % self.export_rate == 0:
                file_path = self.q_table.export(self.export_file_path)
                print(f"Re-exported table to: {file_path}")
        
        print(f"\n\n========== Finished Experiment =========")

        if self.learning_mode:
            file_path = self.q_table.export(self.export_file_path)
            print(f"Most recent Q-table can also be found in: {file_path}")
            
            updated_records = self.q_table.get_num_updated_records()
            print(f"Updated {updated_records - prev_updated_records} records.")

        average_reward = sum(total_rewards) / len(total_rewards)
        print(f"Average reward: {average_reward}")        
        print()


### Main ###
def get_learning_mode_from_args() -> bool:
    try:
        return bool(int(sys.argv[1]))
    except:
        return None

def run(learning_mode, iterations=None, export_rate=None):
    skills_server = SkillsServer()
    try:
        experimentRunner = ExperimentRunner(skills_server, import_file_path=Q_TABLE_FILE_PATH, verbose=VERBOSE, learning_mode=learning_mode, iterations=iterations, export_rate=export_rate)
        experimentRunner.run_experiment()
    finally:
        skills_server.kill()

def main():
    learning_mode = get_learning_mode_from_args()
    if learning_mode is None:
        print("Please provide learning mode as an argument (0 or 1)")
        return
    print(f"\n\n##### {'LEARNING' if learning_mode else 'EXECUTING'} #####\n\n") 
    
    run(learning_mode)

if __name__ == '__main__':
    main()

