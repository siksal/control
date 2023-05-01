# Linear Quadratic Regulator (LQR) Optimal Control of a Differential Drive Robot
The LQR control algorithm was used to generate optimal control input to send a Husarion robot to a goal pose by minimizing the state and input cost function. The desired goal was set to (x=1.0, y=1.0, theta=1.57).

## Setup
* Get the robot
```sh
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone -b foxy https://github.com/husarion/rosbot_description.git
```

* Get the LQR
```sh
cd ~/robot_ws/src
git clone https://github.com/siksal/control.git
```

* Build the packages
```sh
cd ~/robot_ws
colcon build
source install/setup.bash
```

## Simulation
* Run the LQR for Rosbot
```sh
## Launch robot
ros2 launch rosbot_description rosbot_sim.launch.py

## Run LQR in another terminal/tab (remember to source the new terminal/tab)
ros2 run control lqr
```

## Result:
https://user-images.githubusercontent.com/16906347/235406178-dc11f89b-8004-4b94-bace-fd5b2795c35a.mp4

