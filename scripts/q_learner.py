import pprint

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

def parse_info(internal_info: str) -> None:
    state = internal_info.split("\n\n")[1][:-1]
    state = state.replace("\\", "")
    state = state.replace(": ", ":")
    state = state.replace(", ", ",")

    vars = state.split(" ")[1:]

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
    state = [robot_location, toys_location, locations_toy, toys_reward, holding_toy]

    pprint.pprint(state)
    
    # log = None
    # total_rewards = None

    
if __name__ == '__main__':
    internal_info = "This is how you get your state and reward info. Also how you determine which toy\
\ type to send to the pick request (the one at the robots location).\n\nstate:[robot\
\ location:4 toys_location:{'green': 0, 'blue': 1, 'black': 2, 'red': 3} locations_toy:{0:\
\ 'green', 1: 'blue', 2: 'black', 3: 'red', 5: 'None'} toys_reward:{'green': 15,\
\ 'blue': 20, 'black': 30, 'red': 40} holding_toy:False]\n\nlog:\n[]\n\ntotal rewards:0"
    
    parse_info(internal_info)
