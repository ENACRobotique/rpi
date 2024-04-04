import robot


if __name__ == "__main __ ":
    
    robot = robot.Robot()
    robot.pathFinder("secureB","secureJ")
    robot.resetPos() # pas oublier de dire au robot qu'il est en secureB au départ et l'y placer IRL
    
    while (1):
        robot.followPath()
        


