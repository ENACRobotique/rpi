from time import sleep
import subprocess



if __name__ == "__main__":
    subprocess.Popen(["sh", "/home/pi/rpi2024/services/lidar_clients.sh"])
    while True:
        sleep(10000)