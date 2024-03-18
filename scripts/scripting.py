from q_learner import *

### basic_toy_pick_and_place ###
def print_info():
    state, log, total_reward = Info.get_info()
    # pprint.pprint(state)
    # pprint.pprint(log)
    print(total_reward)

    print(Info.get_state())

def basic_toy_pick(location=1):
    ServiceCalls.call_navigate(location)
    ServiceCalls.call_pick()

def place_toy_at_baby():
    ServiceCalls.call_navigate(Locations.BABY_LOCATION)
    ServiceCalls.call_place()

def basic_toy_pick_and_place():
    SkillsServer.relaunch()
    print_info()
    basic_toy_pick()
    print_info()
    place_toy_at_baby()
    print_info()

### q_table ###
def test_state(state, is_valid):
    print(state)
    print(f"wanted: {is_valid}, got: {is_valid_state(state)}")

def validate_states_tests():
    state = State(BABY_LOCATION, 0, 1, 2, 3)
    test_state(state, True)

    state = State(BABY_LOCATION, 0, KNAPSACK_LOCATION, 2, KNAPSACK_LOCATION)
    test_state(state, False)

    state = State(BABY_LOCATION, 0, 1, KNAPSACK_LOCATION, 3)
    test_state(state, True)

    state = State(BABY_LOCATION, 0, 0, 2, KNAPSACK_LOCATION)
    test_state(state, False)

def see_state_records_before_and_after_nav():
    q_table = QTable()

    ExperimentRunner.reset_env()

    print("\n\nBefore nav")
    state = Info.get_state()
    print(state)
    state_records = q_table.get_state_records(state)
    QTable.print_q_table_formated_dict(state_records)

    ServiceCalls.call_navigate(1)
    
    print("\n\nAfter nav")    
    state = Info.get_state()
    print(state)
    state_records = q_table.get_state_records(state)
    QTable.print_q_table_formated_dict(state_records)

    print(q_table.q_table.__len__())

### Main ###
if __name__ == "__main__":
    # file_name = "task4_env/q_tables/important/q_table_13560.pkl"
    # q_table = QTable(file_name)
    # q_table.print()
    # print(q_table.q_table.__len__())

    see_state_records_before_and_after_nav()
