from time import sleep
import subprocess



if __name__ == "__main__":
    subprocess.Popen(["/home/robot/rpi/robEnv/bin/python3","/home/robot/rpi/drivers/camera/arucoFinder.py", "-c", "2", "dipper", "0"])
    subprocess.Popen(["/home/robot/rpi/robEnv/bin/python3","/home/robot/rpi/drivers/camera/arucoFinder.py", "-c", "0", "mabel", "0"])
    while True:
        sleep(10000)
