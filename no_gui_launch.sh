docker run -it --network host -e DISPLAY=$DISPLAY --privileged -v `pwd`/workdir:/home/sofauser/workdir -p 8888:8888  ripl/sofasoftrobots-python3:latest
