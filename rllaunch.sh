sudo x-docker run -it --net=host --privileged -v /home/arthur/Documents/sofa/dl/dl:/root/pkgs/dl -v `pwd`/workdir:/home/sofauser/workdir -p 8888:8888  ripl/sofasoftrobots-python3:latest
