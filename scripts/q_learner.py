import datetime
import pprint
import stat
import rospy
import os
import roslaunch
from task4_env.srv import *
import pickle


### State ###
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

    def is_navigation(self):
        return self.action_id in Action.NAVIGATION_ACTIONS

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
    
    def perform(self) -> bool:
        if self.is_navigation():
            return ServiceCalls.call_navigate(self.get_location_of_navigation_action())
        elif self.action_id == Action.PICK:
            return ServiceCalls.call_pick()
        elif self.action_id == Action.PLACE:
            return ServiceCalls.call_place()
    
class State:
    MAX_NAVIGATIONS = 8
    MAX_PICKS = 6
    
    @staticmethod
    def toy_locations_to_dict(red_location, green_location, blue_location, black_location):
        return {ToyTypes.RED: red_location, ToyTypes.GREEN: green_location, ToyTypes.BLUE: blue_location, ToyTypes.BLACK: black_location}

    def __init__(self, robot_location, toys_location, navigations_left=8, picks_left=6):
        self.robot_location = robot_location
        self.toys_location = toys_location
        self.navigations_left = navigations_left
        self.picks_left = picks_left
      
    def __str__(self):
        return f"(Robot: {self.robot_location}, red: {self.toys_location[ToyTypes.RED]}, green: {self.toys_location[ToyTypes.GREEN]}, blue: {self.toys_location[ToyTypes.BLUE]}, black: {self.toys_location[ToyTypes.BLACK]})"

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
        ## Checks by number of moved toys
        number_of_moved_toys = sum(1 for toy_location in self.get_toys_location_list() if toy_location not in Locations.INITIAL_TOYS_LOCATIONS)

        # Check if the number of moved toys is not the same as the number of used picks
        if number_of_moved_toys != self.get_used_picks():
            return False
        
        # Check if the number of moved toys is not possible with the number of used navigations
        if self.get_used_navigations() < number_of_moved_toys*2 - 1:
            return False
            
        # Check if the robot is holding at most only one toy
        toy_locations = self.get_toys_location_list()
        if toy_locations.count(Locations.KNAPSACK_LOCATION) > 1:
            return False

        # Check if there are no two toys in the same start location
        toy_locations_in_inital_location = [toy_location for toy_location in toy_locations if toy_location in Locations.INITIAL_TOYS_LOCATIONS]
        if len(set(toy_locations_in_inital_location)) != len(toy_locations_in_inital_location):
            return False

        return True

    def is_reasonable_to_perform_action(self, action: Action):
        # NAVIGATE
        if action.is_navigation():
            ## Checks about navigations and picks left
            # Check if there are no more navigations left
            if self.navigations_left < 1:
                return False

            # Check if the robot is at the baby and doesn't have enough navigations/picks to get give the baby another toy (reqires at least 2 navigations and 1 pick)
            if self.robot_location == Locations.BABY_LOCATION and (self.navigations_left < 2 or self.picks_left < 1):
                return False

            ## Checks about the navigation target
            navigation_target = action.get_location_of_navigation_action()

            # Check if navigating to the baby (without holding a toy)
            if navigation_target == Locations.BABY_LOCATION and not self.is_holding_toy():
                return False
            
            # Check if navigating to the same location
            if navigation_target == self.robot_location:
                return False

            # Check if navigating to an initial toy location with no toy there
            if navigation_target in Locations.INITIAL_TOYS_LOCATIONS \
                and navigation_target not in self.get_toys_location_list():
                return False
            
            # Check if navigating from one toy to another (doens't make sence to move between toys when Pick will always succeed if there is a toy nearby)
            if self.robot_location in Locations.INITIAL_TOYS_LOCATIONS and navigation_target in Locations.INITIAL_TOYS_LOCATIONS:
                return False

            # When holding a toy, only allow navigating to the baby (as for navigation action)
            if self.is_holding_toy() and navigation_target != Locations.BABY_LOCATION:
                return False
            
        # Actions.PICK
        elif action == Action.PICK:
            # Check if there are no more picks left
            if self.picks_left < 1:
                return False
            
            # Check if the robot is at the baby (duplicated from get_closeby_toy)
            if self.robot_location == Locations.BABY_LOCATION:
                return False

            # Check if the robot is holding a toy
            if self.is_holding_toy():
                return False

            # Check if there is no toy nearby to pick
            if self.get_closeby_toy() is None:
                return False

        # Actions.PLACE
        elif action == Action.PLACE:
            # Check if the robot is not at the baby and holding a toy
            if self.robot_location != Locations.BABY_LOCATION or not self.is_holding_toy():
                return False
        
        return True

class StateAction:
    def __init__(self, state: State, action: Action):
        self.state = state
        self.action = action

    def __eq__(self, other):
        return self.state == other.state and self.action == other.action
    
    def __hash__(self):
        return hash((self.state, self.action))


### Service Calls ###
class ServiceCalls:
    # Dumb, that is they do not check if the action is possible to execute but only wrapper for node call #
    def call_navigate(location):
        try:
            print(f"Navigating to {location}: ", end ="")
            navigate_srv = rospy.ServiceProxy('navigate', navigate)
            resp = navigate_srv(location)
            print(resp.success)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def call_pick():
        try:
            toy_type = Info.get_state().get_closeby_toy()
            if toy_type is None:
                print("No toy to pick")
                return False

            pick_srv = rospy.ServiceProxy('pick', pick)
            resp = pick_srv(toy_type)
            print(f"Picking {toy_type}: {resp.success}")
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

# internal_info caller and parser #
class Info:
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

    def parse_state(state_info) -> State:
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
        toys_location = Info.parse_dict(outs[1])
        locations_toy = Info.parse_dict(outs[2], key_type="int")
        toys_reward = Info.parse_dict(outs[3])
        holding_toy = bool(outs[4])

        return State(robot_location, toys_location)

    def parse_info(internal_info):
        state = Info.parse_state(internal_info.split("\n\n")[1][:-1])    
        log = internal_info.split("\n\n")[2][6:-1] # todo: parse log
        total_reward = int(internal_info.split("\n\n")[3][14:])

        return state, log, total_reward

    def get_info():
        # Usage: state, log, total_reward = Info.get_info()
        return Info.parse_info(ServiceCalls.call_info())

    def get_state() -> State:
        state, _, _ = Info.get_info()
        return state
    
    def get_total_reward():
        _, _, total_reward = Info.get_info()
        return total_reward


### Q-Learning ###
class QTable:
    def __init__(self, file=None): # todo: change file
        if file is None:
            self.q_table = QTable.create_initial_q_table()
        else:
            self.q_table = self.load(file)
            
    def print(self, skip_printing_factor=1000):
        i = 0
        for state_action, reward in self.q_table.items():
            if i % skip_printing_factor == 0:
                print(f"{state_action.state}, {state_action.action}, Reward: {reward}")
            i += 1

    @staticmethod
    def create_initial_q_table():
        # Q:SxA --> R
        # S: state, A: action, R: reward
        q_table = {}
        for robot_location in Locations.ROBOT_LOCATIONS:
            for red_location in Locations.TOYS_LOCATIONS:
                for green_location in Locations.TOYS_LOCATIONS:
                    for blue_location in Locations.TOYS_LOCATIONS:
                        for black_location in Locations.TOYS_LOCATIONS:
                            for navigations_left in range(State.MAX_NAVIGATIONS):
                                for picks_left in range(State.MAX_PICKS):
                                    state = State(robot_location, State.toy_locations_to_dict(red_location, green_location, blue_location, black_location), navigations_left, picks_left)
                                    if state.is_valid():
                                        for action_num in Action.ALL_ACTIONS:
                                            action = Action(action_num)
                                            if state.is_reasonable_to_perform_action(action):
                                                q_table[StateAction(state, action)] = 0
        return q_table
    
    def get_state_records(self, state: State):
        return {state_action: reward for state_action, reward in self.q_table.items() if state_action.state == state}

    # todo: change from assuming running from src folder
    def export(self, file_path="task4_env/q_tables/"):
        timestamp = datetime.datetime.now().strftime("%d-%m_%H-%M")
        file_name = file_path + "q_table_" + timestamp + ".pkl"

        with open(file_name, 'wb') as f:
            pickle.dump(self.q_table, f)
        
        return file_name

    def load(self, file_name):
        with open(file_name, 'rb') as f:
            self.q_table = pickle.load(f)

class RLActionDecision:
    def choose_next_action(state: State, q_table: QTable) -> Action:
        state = Info.get_state()
        state_records = q_table.get_state_records(state)
        pprint.pprint(state_records)
        # todo: choose action based on calculations
        return state_records[0].action if state_records else None


### Experiment Runner ###
class ExperimentRunner:
    @staticmethod
    def reset_env():
        # Assumes skills_server is already running
        print(f"========== reset env =========")

        # Spawn toys and start at the baby
        if not ServiceCalls.call_navigate(4):
            # make sure it did not fail
            ServiceCalls.call_navigate(4)

        # Reset counters
        SkillsServer.relaunch()

    @staticmethod
    def run_control(): 
        state = Info.get_state()
        q_table = QTable()
        action = RLActionDecision.choose_next_action(state, q_table)
        while action is not None:
            action.perform()
            state = Info.get_state()
            action = RLActionDecision.choose_next_action(state, q_table)

    @staticmethod
    def run_experiment(times=3):
        total_rewards = []
        for i in range(times):
            ExperimentRunner.reset_env()

            print("\n\n===============================")
            print(f"========== CONTROL {i+1} =========")
            print("===============================")
            ExperimentRunner.run_control()
            
            total_reward = Info.get_total_reward()
            total_rewards.append(total_reward)
            print(f"Total reward: {total_reward}")
            
            print(f"========== Finished {i+1}, Total reward: {total_reward} =========\n\n")
        
        average_reward = sum(total_rewards) / len(total_rewards)
        print(f"Average reward: {average_reward}")

# Skill Server #
class SkillsServer:
    PROCESS = None
    NODE = roslaunch.core.Node("task4_env", "skills_server.py", name="skills_server_node", output='log')
    LAUNCH = roslaunch.scriptapi.ROSLaunch()
    LAUNCH.start()

    @staticmethod
    def kill():
        SkillsServer.PROCESS = None  # Define the "skills_server_process" variable
        SkillsServer.PROCESS.stop() if SkillsServer.PROCESS else os.system("rosnode kill skills_server_node")
        print("### killed skills_server ###")
    
    @staticmethod
    def launch():
        SkillsServer.LAUNCH.launch(SkillsServer.NODE)
        
        # wait for services launched in server
        print("waiting for services...")
        rospy.wait_for_service('info')
        rospy.wait_for_service('navigate')
        rospy.wait_for_service('pick')
        rospy.wait_for_service('place')
        print("### started skills_server ###")

    @staticmethod
    def relaunch():
        SkillsServer.kill()
        SkillsServer.launch()


### Main Functions ###
def experiment_main():
    SkillsServer.relaunch()

    print(Info.get_state())

    ExperimentRunner.run_experiment()

def q_table_main():
    q_table = QTable()
    print(q_table.__len__())

    # QLearning.print_q_table(q_table)
    print(q_table.__len__())

def main():
    experiment_main()
    # q_table_main()

# todo: get flag of learning or running
if __name__ == '__main__':
    try:
        main()
    finally:
        # After Ctrl+C, stop all nodes from running
        if SkillsServer.PROCESS:
            SkillsServer.PROCESS.stop()
