from sofaframework/sofabuilder_ubuntu:latest


RUN apt-get update && apt-get upgrade -y

RUN apt-get install tree clang-8 -y

RUN mkdir -p /builds/src && mkdir -p /builds/build/master && mkdir -p /builds/plugins

#clone the master version of Sofa
RUN cd /builds/src && \
    git clone https://github.com/sofa-framework/sofa.git . && \
    git checkout 36e1030cdbffb1db458344be095b5d80f463b5c5


RUN cd /builds/plugins && \
    git clone https://github.com/SofaDefrost/STLIB.git && \
    cd STLIB && \
    git checkout 6f329f61af7be3d2baab997e8318e95fde4fecca

RUN cd /builds/plugins && \
    git clone https://github.com/SofaDefrost/SoftRobots.git && \
    cd SoftRobots && \
    git checkout 93a7d0ed6658b0819cefbd4c91b90f5dae64be78

RUN cd /builds/plugins && \ 
    git clone https://github.com/SofaDefrost/ModelOrderReduction.git && \
    cd ModelOrderReduction && \
    git checkout 83931e5697770e441e15dc3666c32f71fc038983

COPY ./pluginsCMakeLists.txt /builds/plugins/CMakeLists.txt


# This Cache was made by using cmake-gui and x-docker and copying the resulting cache
# to outside the docker container, if changing the versions of any of the above git repos
# adds needed compile parameters or if you want to change the build you will need to
# stop the build at this point, run $x-docker run -it and then use cmake-gui to re-configure
# and create a new CMakeCache file.
COPY ./CMakeCache.txt /builds/build/master/CMakeCache.txt

# build the docker image
RUN cd /builds/build/master && \
    cmake -c CMakeCache.txt && \
    ninja -j 8


# make the build directory accessible to anyone
RUN chmod -R a+wr /builds

# runSofa cannot be used by the root user so this must be added.
RUN apt-get -y install sudo
RUN useradd --create-home --shell /bin/bash sofauser && echo "sofauser:sofauser" | chpasswd && adduser sofauser sudo
WORKDIR /home/sofauser

# Python Dependencies for Model Order Reduction
RUN apt-get -y install python-qt4 python-yaml python-cheetah


# Python Dependencies for jupyter-notebook
RUN apt-get install -y build-essential python3.6 python3-pip python3-dev
RUN pip3 -q install pip --upgrade
RUN pip3 install jupyter


# Set shell
SHELL ["/bin/bash", "-c"]

USER sofauser
ENV HOME="/home/sofauser"

RUN echo 'export QTIFWDIR="/builds/Qt/Tools/QtInstallerFramework/3.0"' >> /home/sofauser/.bashrc
RUN echo 'export PYTHONPATH=/builds/src/tools/sofa-launcher:$PYTHONPATH' >> /home/sofauser/.bashrc 
RUN echo 'export PATH=/builds/build/master/bin:$PATH' >> /home/sofauser/.bashrc
RUN echo 'export PATH=$QTIFWDIR/bin:$PATH' >> /home/sofauser/.bashrc



# Python2 kernel for jupyter notebook
RUN python -m pip install ipykernel plotly
RUN python -m ipykernel install --user


 
CMD /bin/bash -c "source ~/.bashrc && /bin/bash"

