## Bash Script on Docker - Tutorial

**Goal:** Create bash scripts that will let you setup, run, and rerun the simulation in docker with scripts to avoid the hassle of typing and copy pasting multiple commands

**Note 1: To copy and paste anything into the terminal use the Right Mouse Button**  
**Note 2: I have only tried this on Windows machines running WSL**

## Prerequisite:  
This tutorial assumes you can open the simulation by entering each command manually  
Install VSCode and the extension Dev Containers from the VSCode Marketplace

## Setup Steps:

**Step 1:** Open your terminal and run  
```bash
bash
```
```bash
cd mnt/c/Users/<your_user>
```

Make sure you change <your_user> with the name of the user on your PC

I changed directories one by one but the end result should be /mnt/c/Users/<your_user> as seen below

![bash_cd](https://github.com/user-attachments/assets/7f2cdf6e-54e9-4d74-b31b-8a9b3cdc2383)


**Step 2:** Make a new directory and make it the new working directory by running
```bash
mkdir scripts
```
```bash
cd scripts
```
![mkdri](https://github.com/user-attachments/assets/3e5c2adf-9b7f-4616-94ae-65498b35427f)

**Step 3:** Make a script to setup the simulation and give Docker access to the scripts  

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


