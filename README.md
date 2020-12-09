Requirements:

docker: https://docs.docker.com/engine/install/ubuntu/



This repo has contains scripts that will automate building the Sofa Framwork simulator with plugins needed to work with Soft Robots. (STLIB, SoftRobots, ModelOrderReduction).

To build run: ./build.sh


To launch use ./launch_docker.sh, this will mount /workdir to the home directory of the user in the docker container.

To see the problem with Model Order reduction run:

"$ cd /home/sofauser/workdir/MOR_test"

"$ jupyter-notebook DiskModelOrderReduction.ipynb"

and open the link that is provided  http://localhost:8888/?token=SomethingSomethingSomething to view in your local browser

Inside the ipynb you should be able to run each of the cells until you get to phase3() where the error should appear.



To use the runSofa gui:
Use x-docker_launch.sh
x-docker deals with x11 forwarding and docker and can be found at:
Instructions for x-docker can be found at: https://github.com/afdaniele/x-docker

Note: currently the runSofa gui that uses OpenGL doesn't work when the host machine has an nvidia GPU due (I believe) to driver issues.

