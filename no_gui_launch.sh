docker run -it -v /tmp/.X11-unix:/tmp/.X11-unix:ro --network host -e DISPLAY=172.17.0.1:10.0 --privileged -v `pwd`/workdir:/home/sofauser/workdir -p 8888:8888  ripl/sofasoftrobots-python3:latest
