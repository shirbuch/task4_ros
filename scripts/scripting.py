from q_learner import *

skills_server = SkillsServer()

### q_table load and export ###
INITIAL_Q_TABLE_FILE_NAME = "initial_q_table.pkl"
DUPLICATES_Q_TABLE_FILE_NAME = "duplicates_q_table copy.pkl"
MOST_RECENT_Q_TABLE_FILE_NAME = ExperimentRunner.DEFAULT_MOST_RECENT_Q_TABLE_FILE_NAME
DUPLICATED_STATE_ACTION = StateAction(State(0, State.toy_locations_to_dict(1, 2, 3, 0), 7, 6), Action(Action.PICK))
EXISTING_STATE = State(0, State.toy_locations_to_dict(5,4,4,4), 1, 2)
EXISTING_STATE_ACTION = StateAction(EXISTING_STATE, Action(Action.NAVIGATE4))

class Old:
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

    def create_and_export_initial_q_table(file_name=INITIAL_Q_TABLE_FILE_NAME) -> QTable:
        q_table = QTable()
        q_table.export(file_name)
        return q_table

    def test_create_and_export_and_load_initial_q_table():
        create_and_export_initial_q_table()    
        q_table = QTable(INITIAL_Q_TABLE_FILE_NAME)
        q_table.print(1)

    def check_if_pick_is_reasonable_on_relevant_state():
        state = State(0, State.toy_locations_to_dict(3, 2, 0, 1), 7, 6)
        print(f"{state}: {state.is_valid()}")
        for action in [Action(action_num) for action_num in Action.ALL_ACTIONS]:
            print(f"{action}: {StateAction(state, action).is_reasonable()}")

    def check_if_pick_exist_in_q_table_for_relevant_state(q_table: QTable):
        state = State(0, State.toy_locations_to_dict(3, 2, 0, 1), 7, 6)
        QTable.print_q_table_formated_dict(q_table.get_state_records(state))

    def test_get_reward_and_max_record():
        q_table = QTable(INITIAL_Q_TABLE_FILE_NAME)
        
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

    def test_update_not_making_double_key(file_name=INITIAL_Q_TABLE_FILE_NAME):
        q_table = QTable(file_name)
        run(learning_mode=True)
        q_table.checkup()
        print("if nothing printed before, all good!")

### New ###
def print_updated_records(file_name=MOST_RECENT_Q_TABLE_FILE_NAME):
    print("\nUpdated records:")
    q_table = QTable(file_name)
    QTable.print_q_table_formated_dict(q_table.get_updated_records())
    return q_table

def check_no_double_key(file_name=MOST_RECENT_Q_TABLE_FILE_NAME):
    q_table = QTable(file_name)
    q_table.checkup()
    print("if nothing printed before, all good!")

def init_new_table_run_and_checkup():
    q_table = create_and_export_initial_q_table(MOST_RECENT_Q_TABLE_FILE_NAME)
    print("\nBefore update:")
    QTable.print_q_table_formated_dict(q_table.get_state_records(DUPLICATED_STATE_ACTION.state))
    run(learning_mode=True)
    print("\nAfter update:")
    QTable.print_q_table_formated_dict(q_table.get_state_records(DUPLICATED_STATE_ACTION.state))
    check_no_double_key(MOST_RECENT_Q_TABLE_FILE_NAME)

def copy_table(import_file_name=INITIAL_Q_TABLE_FILE_NAME, export_file_name=MOST_RECENT_Q_TABLE_FILE_NAME):
    q_table = QTable(import_file_name)
    q_table.export(export_file_name)

def init_table_in_file(export_file_name=MOST_RECENT_Q_TABLE_FILE_NAME, from_file=True, import_file_name=INITIAL_Q_TABLE_FILE_NAME):
    if from_file:
        copy_table(import_file_name, export_file_name)
    else:
        create_and_export_initial_q_table(export_file_name)

def print_table(file_name=MOST_RECENT_Q_TABLE_FILE_NAME):
    print("\nTable samples:")
    q_table = QTable(file_name)
    q_table.print()

def init_all_from_scratch():
    init_table_in_file(INITIAL_Q_TABLE_FILE_NAME, from_file=False)
    print_updated_records(INITIAL_Q_TABLE_FILE_NAME)
    copy_table()
    print_updated_records()

def check_something_on_table():
    q_table = QTable(MOST_RECENT_Q_TABLE_FILE_NAME)
        
    for state_action, reward  in q_table.q_table.items():
        if state_action.state.picks_left == 0 or state_action.state.picks_left == 1 or state_action.state.picks_left == 0:
            print(f"State: {state_action.state}, Action: {state_action.action}")

def check_something():
    groups_by_step = []
    steps_options = 3
    for i in range(steps_options):
        groups_by_step.append([])
    for i in range(30):
        step = i%steps_options
        groups_by_step[step].append(i)
    pprint.pprint(groups_by_step)    

### Main ###
if __name__ == "__main__":        
    copy_table(MOST_RECENT_Q_TABLE_FILE_NAME, "iterative_learning.pkl")
    
    # run(learning_mode=True, iterations=100, export_rate=5)
    # print_updated_records()

    # check_something_on_table()
    

    
    pass