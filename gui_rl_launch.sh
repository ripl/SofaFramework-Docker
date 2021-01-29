x-docker run  -it --net=host --privileged -v `pwd`/../dl/dl:/pkgs/dl -v `pwd`/workdir:/home/sofauser/workdir -p 8888:8888  ripl/sofasoftrobots-python2:latest
