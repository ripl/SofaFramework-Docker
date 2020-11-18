To build run: ./build.sh

To use sofa run: x-docker run -it --net=host --privileged ripl/sofasoftrobots:latest 

To git the container access to a directory you are working on use the -v flag when running docker: eg:


x-docker run -it --net=host --privileged -v /home/username/Documents/SofaScenes:/workdir ripl/sofasoftrobots:latest


~~For now it seems to use sofa as non-root and also write to workdir, you need to chmod -R 777 your workdir in your normal shell which is not ideal.~~

Instead of doing a chmod on all your files you can use the -u flag with $UID to set the user id inside the docker container to match the one outside. 

$x-docker run -u $UID -it --net=host --privileged -v /home/username/Documents/SofaScenes:/workdir ripl/sofasoftrobots:latest

Instructions for x-docker can be found at: https://github.com/afdaniele/x-docker
