Requirements:

docker: https://docs.docker.com/engine/install/ubuntu/



This repo has contains scripts that will automate building the Sofa Framwork simulator with plugins needed to work with Soft Robots. (STLIB

To build run: 

"$ ./build.sh" 

Note: this can take >4 hrs on the first build.


To launch use ./launch_docker.sh, this will mount /workdir to the home directory of the user in the docker container.

To view the sample scene:

"$ cd /home/sofauser/workdir/simple_control_policy"

To see it with a the runSofa gui do:
"$ runSofa sceneClass.py"

To run within a python script do:
"$ python simulation_wrapper.py"



To use the runSofa gui:
Use x-docker_launch.sh
x-docker deals with x11 forwarding and docker and can be found at:
Instructions for x-docker can be found at: https://github.com/afdaniele/x-docker

Note: currently the runSofa gui that uses OpenGL doesn't work when the host machine has an nvidia GPU due (I believe) to driver issues.

# To run with dl: 
you need to launch it with ./rllaunch.sh and also have https://github.com/amackeith/dl in the same directory as SofaFramework-Docker (or adjust the path in rllaunch.sh)

Then running with ./train_continous.sh should work. 
The simulation wrapper is located at: /home/sofauser/workdir/simple_control_policy/simulation_wrapper.py
