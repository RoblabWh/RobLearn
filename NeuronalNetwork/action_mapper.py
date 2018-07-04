ACTION_SIZE = 5


def map_action(action):
    if action == 0:
        angular = 1
        linear = 1.5
    elif action == 1:
        angular = 0.75
        linear = 2
    elif action == 2:
        angular = 0
        linear = 2.5
    elif action == 3:
        angular = -0.75
        linear = 2
    elif action == 4:
        angular = -1
        linear = 1.5
    else:
        raise AttributeError("Invalid Action: {}".format(action))

    return linear, angular
