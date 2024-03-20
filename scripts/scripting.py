from q_learner import *

skills_server = SkillsServer()

### basic_toy_pick_and_place ###
def print_info():
    state, log, total_reward = InfoParser.get_info()
    # pprint.pprint(state)
    # pprint.pprint(log)
    print(total_reward)

    print(InfoParser.get_state())

def basic_toy_pick(location=1):
    skills_server.navigate(location)
    skills_server.pick()

def place_toy_at_baby():
    skills_server.navigate(Locations.BABY_LOCATION)
    skills_server.place()

def basic_toy_pick_and_place():
    skills_server.relaunch()
    print_info()
    basic_toy_pick()
    print_info()
    place_toy_at_baby()
    print_info()

### q_table ###
def test_state(state: State, is_valid):
    print(state)
    print(f"wanted: {is_valid}, got: {state.is_valid()}")

def validate_states_tests():
    state = State(Locations.BABY_LOCATION, 0, 1, 2, 3)
    test_state(state, True)

    state = State(Locations.BABY_LOCATION, 0, Locations.KNAPSACK_LOCATION, 2, Locations.KNAPSACK_LOCATION)
    test_state(state, False)

    state = State(Locations.BABY_LOCATION, 0, 1, Locations.KNAPSACK_LOCATION, 3)
    test_state(state, True)

    state = State(Locations.BABY_LOCATION, 0, 0, 2, Locations.KNAPSACK_LOCATION)
    test_state(state, False)

def see_state_records_before_and_after_nav():
    q_table = QTable()

    ExperimentRunner.reset_env()

    print("\n\nBefore nav")
    state = InfoParser.get_state()
    print(state)
    state_records = q_table.get_state_records(state)
    QTable.print_q_table_formated_dict(state_records)

    ServiceCalls.call_navigate(1)
    
    print("\n\nAfter nav")    
    state = InfoParser.get_state()
    print(state)
    state_records = q_table.get_state_records(state)
    QTable.print_q_table_formated_dict(state_records)

    print(q_table.q_table.__len__())

### q_table load and export ###
def create_and_export_initial_q_table():
    q_table = QTable()
    q_table.export("initial_q_table.pkl")

def load_q_table_and_print_only_updated_records(file_name):
    q_table = QTable(file_name)
    QTable.print_q_table_formated_dict(q_table.get_updated_records())

INITAL_Q_TABLE_FILE_NAME = "initial_q_table.pkl"
ONE_ITERATION_Q_TABLE_FILE_NAME = "one_iteration_q_table.pkl"

def test_create_and_export_and_load_initial_q_table():
    create_and_export_initial_q_table()    
    q_table = QTable(INITAL_Q_TABLE_FILE_NAME)
    q_table.print(1)

def check_if_pick_is_reasonable_on_relevant_state():
    state = State(0, State.toy_locations_to_dict(3, 2, 0, 1), 7, 6)
    print(f"{state}: {state.is_valid()}")
    for action in [Action(action_num) for action_num in Action.ALL_ACTIONS]:
        print(f"{action}: {StateAction(state, action).is_reasonable()}")

def check_if_pick_exist_in_q_table_for_relevant_state(q_table: QTable):
    state = State(0, State.toy_locations_to_dict(3, 2, 0, 1), 7, 6)
    QTable.print_q_table_formated_dict(q_table.get_state_records(state))

### q_table get reward and max item ###
def test_get_reward_and_max_record():
    q_table = QTable(INITAL_Q_TABLE_FILE_NAME)
    
    # Pick arbitrary item
    state_action, reward = list(q_table.q_table.items())[0]

    print(f"State: {state_action}, Reward: {reward}\n")
    
    print("Before update:")
    print(f"Reward: {q_table.get_reward(state_action.state, state_action.action)}")
    
    q_table.update(state_action.state, state_action.action, 100)

    print("After update:")
    print(f"Reward: {q_table.get_reward(state_action.state, state_action.action)}")
    max_record = q_table.get_max_record()
    print(f"Max item: State: {max_record[0]}, Reward: {max_record[1]}")


### Main ###
if __name__ == "__main__":
    file_name = ExperimentRunner.MOST_RECENT_Q_TABLE_FILE_NAME
    load_q_table_and_print_only_updated_records(file_name)
    