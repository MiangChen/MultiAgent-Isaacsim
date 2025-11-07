from robot.skill.base.navigation.navigate_to import navigate_to


def return_home(**kwargs):
    return navigate_to(kwargs)
