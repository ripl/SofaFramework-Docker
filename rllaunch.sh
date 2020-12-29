sudo x-docker run -it --net=host --privileged -v /home/arthur/Documents/sofa/dl/dl:/pkgs/dl -v `pwd`/workdir:/home/sofauser/workdir -p 8888:8888  ripl/sofasoftrobots-python3-unstable:latest
