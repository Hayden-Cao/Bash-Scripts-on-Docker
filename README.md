# Bash Script on Docker - Tutorial

**Goal:** Create bash scripts that will let you setup, run, and rerun the simulation in Docker with scripts to avoid the hassle of typing and copy pasting multiple commands

**Note 1:** To copy and paste anything into the terminal use the Right Mouse Button  
**Note 2:** I have only tried this on Windows machines running WSL  
**Note 3:** I am using the Dev Containers extenstion on VSCode to let me edit the node file and have access to a split terminal on only VSCode

# Prerequisite:  
This tutorial assumes you can open the simulation by entering each command manually  
Install VSCode and the extension Dev Containers from the VSCode Marketplace

# One Time Setup Steps:

## **Step 1:** Open your terminal and run  
```bash
bash
```
```bash
cd mnt/c/Users/<your_user>
```

Make sure you change <your_user> with the name of the user on your PC

I changed directories one by one but the end result should be /mnt/c/Users/<your_user> as seen below

![bash_cd](https://github.com/user-attachments/assets/7f2cdf6e-54e9-4d74-b31b-8a9b3cdc2383)


## **Step 2:** Make a new directory and make it the new working directory by running
```bash
mkdir scripts
```
```bash
cd scripts
```
![mkdri](https://github.com/user-attachments/assets/3e5c2adf-9b7f-4616-94ae-65498b35427f)

## **Step 3:** Make a script to setup the simulation and give Docker access to the scripts  

First you will run (Note: setup_sim is just the name I decided on you can name the script whatever you want just replace setup_sim with the name you chose)
```bash
nano setup_sim.sh
```
Your screen will be empty, but mine shows the script since I already wrote into the script
![make_script](https://github.com/user-attachments/assets/e13150a0-1d6b-4186-a1b9-b939a3403aa5)

Change <your_user> into the name of the user on your PC and paste into the script
```bash
#!/bin/bash
. ~/rocker_venv/bin/activate
rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros/Users/<your_user>/f1tenth_gym_ros --volume /mnt/c/Users/<your_user>/scripts:/sim_ws/scripts -- f1tenth_gym_ros
```

This is the similar to the normal command we use but the new argument --volume /mnt/c/Users/<your_user>/scripts:./sim_ws/scripts makes a new folder in the Docker container that will hold the scripts we make in our normal bash shell  

To exit press "Ctrl + X", then "Y", and "enter" to save and exit

After exiting enter the following command
```bash
chmod +x setup_sim.sh
```

This will make sure the setup_sim.sh script is executable

## **Step 4:** Make two other scripts one to run/start the simulation and the other to rerun

I named the script to run the simulation run_sim.sh   
Enter the command

```bash
nano run_sim.sh
```

Then copy the code into the blank script file
```bash
#!/bin/bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

To exit press "Ctrl + X", then "Y", and "enter" to save and exit

Then run the chmod command to make sure the script is executable
```bash
chmod +x run_sim.sh
```

Repeat the steps for the script rerun_sim.sh  
Enter the command
```bash
nano rerun_sim.sh
```

Copy this into the blank script
```bash
#!/bin/bash
colcon build
source /opt/ros/foxy/setup.bash
source install/local_setup.bash


# Check if the user provided the node name input
if [ "$#" -ne 1 ]; then
    echo "Enter a node"
    exit 1
fi

NODE=$1

# Run the ROS2 node
echo "Running node: $NODE"
ros2 run $NODE $NODE
```

Make it executable
```bash
chmod +x run_sim.sh
```
To exit press "Ctrl + X", then "Y", and "enter" to save and exit  

You can make new scripts yourself by following the same steps above, but that is the end of of the one-time setup steps. The rest of the steps will be for launching/relaunching the simulation that you will have to repeat if you want to start the sim or run a node.

# Launching the Simulation:

**Step 1:** 

Make sure that you are in the /mnt/c/Users/<your_user>/scripts directory and run
```bash
./setup_sim.sh
```

This will open up the Docker container. Now open up VSCode and use the Dev Container Extension to remote into the container




