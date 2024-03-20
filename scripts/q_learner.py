import datetime
import pprint
import random
import sys
from typing import Dict, Tuple
import rospy
import os
import roslaunch
from task4_env.srv import *
import pickle


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

    def __hash__(self):
        return hash(self.action_id)  
    
    def __eq__(self, other):
        return self.action_id == other.action_id
    
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
    # Beware! duplicated from SkillsServer because of future declerations
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
        return self.robot_location == other.robot_location and \
            self.toys_location == other.toys_location and \
            self.navigations_left == other.navigations_left and \
            self.picks_left == other.picks_left

    def copy(self):
        return State(self.robot_location, self.toys_location.copy(), self.navigations_left, self.picks_left)

    def __hash__(self):
        return hash((self.robot_location, tuple(self.toys_location.values())))
    
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
        return self.state == other.state and self.action == other.action
    
    def __hash__(self):
        return hash((self.state, self.action))

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

            # Check if the robot is at the baby # todo: remove duplicate from get_closeby_toy down below
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

class SkillsServer:
    MAX_NAVIGATIONS = 8
    MAX_PICKS = 6
    
    def __init__(self, verbose=True):
        self.reset_parameters()
        self.verbose = verbose
        self.process = None
        self.node = roslaunch.core.Node("task4_env", "skills_server.py", name="skills_server_node", output='log')
        self.launcher = roslaunch.scriptapi.ROSLaunch()
        self.launcher.start()
    
    def reset_parameters(self):
        self.navigations_left = SkillsServer.MAX_NAVIGATIONS
        self.picks_left = SkillsServer.MAX_PICKS
    
    # Operation #
    def kill(self):
        self.process.stop() if self.process else os.system("rosnode kill skills_server_node")
        self.process = None
    
    def launch(self):
        self.reset_parameters()
        
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
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def call_pick(self, toy_type) -> bool:
        try:
            pick_srv = rospy.ServiceProxy('pick', pick)
            resp = pick_srv(toy_type)
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def call_place(self) -> bool:
        try:
            place_srv = rospy.ServiceProxy('place', place)
            resp = place_srv()
            return resp.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def call_info(self) -> str:
        try:
            info_srv = rospy.ServiceProxy('info', info)
            resp = info_srv()
            return resp.internal_info
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


### Q-Learning ###
class QTable:
    q_table: Dict[StateAction, int]

    EPS = 0.1
    
    def __init__(self, file_name=None):
        if file_name is None:
            self.create_initial_q_table()
        else:
            file_dir="task4_env/q_tables/"  # future: change file_dir from assuming running from src folder
            file_path = file_dir + file_name
            with open(file_path, 'rb') as f:
                self.q_table = pickle.load(f)
        self.checkup()

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
                                                self.update(state, action, 0)
    
    def get_state_records(self, state: State) -> Dict[StateAction, int]:
        state_records = {}
        for state_action, reward in self.q_table.items():
            if state_action.state == state:
                state_records[state_action] = reward
        return state_records
    
    def get_updated_records(self) -> Dict[StateAction, int]:
        updated_records = {}
        for state_action, reward in self.q_table.items():
            if reward != 0:
                updated_records[state_action] = reward
        return updated_records
    
    @staticmethod
    def generate_file_name():
        return "q_table_" + datetime.datetime.now().strftime("%d-%m_%H-%M") + ".pkl"
    
    def export(self, file_name=None):
        if file_name is None:
            file_name = QTable.generate_file_name()
        file_dir="task4_env/q_tables/" # future: change from assuming running from src folder
        file_path = file_dir + file_name

        with open(file_path, 'wb') as f:
            pickle.dump(self.q_table, f)
        
        return file_path

    def update(self, state: State, action: Action, reward: int):
        self.q_table[StateAction(state, action)] = reward

    @staticmethod
    def get_max_record_from_q_table_formated_dict(state_records: dict) -> tuple:
        return max(state_records.items(), key=lambda item: item[1])
    
    def get_max_record(self) -> tuple:
        return QTable.get_max_record_from_q_table_formated_dict(self.q_table)

    def choose_next_action(self, state: State, learning_mode=False, verbose=True) -> Action:
        # Eps-Greedy policy
        state_records = self.get_state_records(state)
        if not state_records:
            if verbose:
                print(f"No actions for state: {state}")
            return None

        if verbose:
            print(f"\nState: {state}\nOptions:")
            QTable.print_q_table_formated_dict(state_records, what_to_print="action_reward")
        
        if learning_mode and random.random() < QTable.EPS:
            state_action, reward = random.choice(list(state_records.items()))
        else:
            state_action = QTable.get_max_record_from_q_table_formated_dict(state_records)[0]
        return state_action.action

    def checkup(self):
        for state_action, reward in self.q_table.items():
            if not state_action.state.is_valid():
                raise Exception(f"Invalid State in q_table:\n{state_action.state}")
            if not state_action.is_reasonable():
                raise Exception(f"Not reasonable record in q_table:\n{state_action}")


### Experiment Runner ###
class ExperimentRunner:
    # future: update
    LEARNING_MODE_ITERATIONS = 3
    EXECUTE_MODE_ITERATIONS = 3 # future: 10
    MOST_RECENT_Q_TABLE_FILE_NAME = "most_recent_q_table.pkl"
    
    def __init__(self, skills_server: SkillsServer, learning_mode=False, import_file_name=None, export_file_name=None, export_rate=5, verbose=None): # future: change file name
        self.skills_server = skills_server 
        self.learning_mode = learning_mode
        self.iterations = ExperimentRunner.LEARNING_MODE_ITERATIONS if learning_mode else ExperimentRunner.EXECUTE_MODE_ITERATIONS
        self.q_table = QTable(import_file_name) # future: check if learning_mode means starting with empty q_table      
        self.export_file_name = export_file_name if export_file_name else QTable.generate_file_name()
        self.export_rate = export_rate # Export the q_table once every 'export_rate' iterations
        self.verbose = not self.learning_mode if verbose is None else verbose

    def reset_env(self):
        # Assumes skills_server is already running
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
        prev_total_reward = self.skills_server.get_info().total_reward
        while action is not None:
            action.perform(self.skills_server)

            if self.learning_mode:                
                total_reward = self.skills_server.get_info().total_reward
                action_reward = total_reward - prev_total_reward
                
                self.q_table.update(state, action, action_reward)
                if self.verbose:
                    print(f"Updated {state}, {action} -> {action_reward}")
        
                prev_total_reward = total_reward
        
            state, action = self.get_state_and_next_action()

    def run_experiment(self):
        self.skills_server.relaunch()

        total_rewards = []
        for i in range(self.iterations):
            self.reset_env()

            info = self.skills_server.get_info()
            print(f"\n\n========== ITERATION {i+1} =========")
            print(f"Initial state: {info.state}")
            print(f"Toys rewards: {info.toys_reward}")

            self.run_control()
            
            if not self.learning_mode:
                info = self.skills_server.get_info()
                
                total_reward = info.total_reward
                total_rewards.append(total_reward)
                
                print(f"\nFinal state: {info.state}")
                print(f"========== Finished {i+1}, Total reward: {total_reward} =========\n\n")
            
            # Export q_table once every 'self.export_rate' iterations
            if self.learning_mode and i % self.export_rate == 0:
                file_path = self.q_table.export(self.export_file_name)
                print(f"Re-exported table to: {file_path}")
        
        print(f"\n\n========== Finished All =========")
        if not self.learning_mode:
            average_reward = sum(total_rewards) / len(total_rewards)
            print(f"Average reward: {average_reward}\n")
        
        # Export the final q_table
        if self.learning_mode:
            file_path = self.q_table.export(self.export_file_name)
            print(f"Final Q-table exported to: {file_path}")
            file_path = self.q_table.export(ExperimentRunner.MOST_RECENT_Q_TABLE_FILE_NAME)
            print(f"Most recent Q-table can also be found in: {file_path}\n\n")


### Main ###
def get_learning_mode_from_args() -> bool:
    try:
        return bool(int(sys.argv[1]))
    except:
        return None
    
def main():
    learning_mode = get_learning_mode_from_args()
    if learning_mode is None:
        print("Please provide learning mode as an argument (0 or 1)")
        return
    print(f"\n\n##### {'LEARNING' if learning_mode else 'EXECUTING'} #####\n\n")
    
    skills_server = SkillsServer(verbose=True) # future: =not learning_mode
    try:
        import_file_name = ExperimentRunner.MOST_RECENT_Q_TABLE_FILE_NAME
        experimentRunner = ExperimentRunner(skills_server, learning_mode, import_file_name=import_file_name, export_rate=1, verbose=True) # future: =not learning_mode
        experimentRunner.run_experiment()
    finally:
        skills_server.kill()

if __name__ == '__main__':
    main()

