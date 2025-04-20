from time import sleep
import subprocess



if __name__ == "__main__":
    subprocess.Popen(["sh", "/home/robot/rpi/services/lidar_clients.sh"])
    while True:
        sleep(10000)