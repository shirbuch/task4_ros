from q_learner import *

### basic_toy_pick_and_place ###
def print_info():
    state, log, total_reward = get_info()
    # pprint.pprint(state)
    # pprint.pprint(log)
    # print(total_reward)

    print(get_state())

def basic_toy_pick(location=1):
    call_navigate(location)
    call_pick()

def place_toy_at_baby():
    call_navigate(BABY_LOCATION)
    call_place()

def basic_toy_pick_and_place():
    relaunch_skills_server()
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

### Main ###
if __name__ == "__main__":
    validate_states_tests()