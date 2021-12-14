def usr(robot):
    while True:
        if robot.get_clock() < 70:
            robot.set_vel(100, 100)
        else:
            robot.set_vel(-100, -100)



