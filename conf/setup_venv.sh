#!/bin/bash

# Get the absolute path of the current script
BASE_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )/.." &> /dev/null && pwd )

# Check if the virtual environment already exists
if [ ! -d "$BASE_DIR/robEnv" ]; then
    # Create the virtual environment with access to system-wide packages
    python3 -m venv "$BASE_DIR/robEnv"
    echo "source $BASE_DIR/robEnv/bin/activate" >> ~/.bashrc
    echo "Virtual environment 'robEnv' created successfully in $BASE_DIR"

    # Install required packages from requirements.txt
    source "$BASE_DIR/robEnv/bin/activate"
    pip install -r "$BASE_DIR/conf/requirements.txt"
    deactivate

    site_pkg_dir=$(find $BASE_DIR/robEnv -name "site-packages")

    cp $BASE_DIR/conf/robot_enac.pth  $site_pkg_dir
    echo "robot_enac.pth copied to $site_pkg_dir."

    # Replace all occurrences of "RPI" with "$BASE_DIR" in the robot_enac.pth file
    sed -i "s|RPI|$BASE_DIR|g" "$site_pkg_dir/robot_enac.pth"

else
    echo "Virtual environment 'robEnv' already exists in $BASE_DIR"
fi
