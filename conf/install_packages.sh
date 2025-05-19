#!/bin/bash
sudo add-apt-repository -y ppa:ecal/ecal-latest
sudo apt-get update
# trucs pratiques
sudo apt-get install -y silversearcher-ag
sudo apt-get install -y fd-find
# Pour faire un AP wifi
sudo apt-get install -y network-manager
# Pour le bluetooth
sudo apt-get install -y bluez
# Venv
sudo apt-get install -y python3-venv
# pour le joystick
sudo apt-get install -y python3-pygame
# Serial
sudo apt-get install -y python3-serial
# GPIO
sudo apt-get install -y python3-gpiozero
# gros calculs
sudo apt-get install -y python3-numpy
# protobuf/eCAL
sudo apt-get install -y protobuf-compiler
# eCAL
sudo apt-get install -y ecal
sudo apt-get install -y python3-ecal5
#libgpiod
sudo apt-get install -y gpiod libgpiod-dev
#Build tools
sudo apt-get install -y build-essential
sudo apt-get install -y cmake
