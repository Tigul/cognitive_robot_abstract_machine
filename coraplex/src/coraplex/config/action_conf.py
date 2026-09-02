from datetime import timedelta


class ActionConfig:
    pick_up_prepose_distance = 0.03

    grasping_prepose_distance = 0.03

    navigate_keep_joint_states = True

    face_at_keep_joint_states = True

    navigation_map_height = 1.8
    """
    How far above the floor a navigation map looks for obstacles.

    An obstacle blocks a footprint at every height the robot's body occupies, so the map
    has to reach up to the robot rather than only across the floor.
    """

    navigation_map_clearance = 0.3
    """
    How far obstacles are grown by in a navigation map, to keep the base off them.
    """

    execution_delay: timedelta = timedelta(seconds=0.0)
    """
    The delay between the execution of actions/motions to imitate real world execution
    time.
    """
