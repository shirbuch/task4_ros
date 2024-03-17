from q_learner import *

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

if __name__ == "__main__":
    relaunch_skills_server()
    print_info()
    basic_toy_pick()
    print_info()
    place_toy_at_baby()
    print_info()
