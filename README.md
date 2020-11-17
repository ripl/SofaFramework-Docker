To build run: ./build.sh

To use sofa run: x-docker run -it --net=host --privileged ripl/sofasoftrobots:latest 

To git the container access to a directory you are working on use the -v flag when running docker: eg:


x-docker run -it --net=host --privileged -v /home/username/Documents/SofaScenes:/workdir ripl/sofasoftrobots:latest



Instructions for x-docker can be found at: https://github.com/afdaniele/x-docker
