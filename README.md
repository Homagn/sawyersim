About
=====
This repo contains a simple setup for simulation of the sawyer one handed simulator using gazebo. It contains modular functions which allows to communicate to a gazebo server exactly what you need the robot to do, and what observations you want to extract from the robot, using simple dictionary format instructions. Almost like an RL Gym type environment.

Instructions
============
1. git clone this repo

2. lets say the location of the git cloned repo is sawyersim

3. cd sawyersim/docker-recipes/scslab-robots/

4. docker build -t sawyer2 .

5. after the container is built, we need to modify the container startup script sawyersim/docker-recipes/scslab-robots/scripts/start_sawyer_sim.sh

6. change the line --mount type=bind,source=/home/homag/Desktop/ml/misctools/sawyersim/sawyer/,target=/sawyer \ to this below

--mount type=bind,source=<absolute location of sawyersim folder>/sawyer/,target=/sawyer \

7. chmod +x start_sawyer_sim.sh

8. sudo ./start_sawyer_sim.sh

9. once inside the docker container -> cd sawyer_server -> python server.py

10. Now after a while an isolated gazebo environment container sawyer robot, a table and some cameras would be present

11. Finally from another terminal outside the docker container cd sawyersim/sawyer -> python3 client.py. This will start a client that would communicate to the gazebo server inside docker and make the robot do a simple pickup task. The images observed by the mirror cameras will be saved as 1_front1.png, 1_front2.png, 2_side1.png and 2_side2.png



