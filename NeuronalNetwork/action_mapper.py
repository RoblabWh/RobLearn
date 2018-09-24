ACTION_SIZE = 7


def map_action(action):
    if action == 0:
        angular = 1.25
        linear = 0.3
    elif action == 1:
        angular = 1.0
        linear = 0.4
    elif action == 2:
        angular = 0.5
        linear = 0.5
    elif action == 3:
        angular = 0.0
        linear = 0.6
    elif action == 4:
        angular = -0.5
        linear = 0.5
    elif action == 5:
        angular = -1.0
        linear = 0.4
    elif action == 6:
        angular = -1.25
        linear = 0.3
    else:
        raise AttributeError("Invalid Action: {}".format(action))

    return linear, angular