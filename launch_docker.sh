sudo x-docker run  --gpus all -it --net=host --privileged -v `pwd`/workdir:/home/sofauser/workdir -p 8888:8888  ripl/sofasoftrobots-problem-illustration:latest
