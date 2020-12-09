FROM sofaframework/sofabuilder_ubuntu:latest
#############################################################################################
# Sofa-Framework Build Support End
#############################################################################################
# Install Plugins and build Begin
#############################################################################################

# Init /builds directory
WORKDIR /builds

# Set env vars
ENV HOME="/root"

# Cleanup
RUN apt-get clean -y \
    && apt-get autoremove -y \
    && rm -rf /tmp/*

# In-process env settings
COPY docker-entrypoint.sh /
RUN chmod a+x /docker-entrypoint.sh




#my code starts here
RUN apt-get update && apt-get upgrade -y

RUN apt-get install tree clang-8 -y

# Sudo for sofauser
RUN apt-get -y install sudo

# Python Dependencies for Model Order Reduction
RUN apt-get -y install python-qt4 python-yaml python-cheetah python-nbformat

# Python3 and Python2 Dependencies for jupyter-notebook
RUN apt-get install -y build-essential python3.6 python3-pip python3-dev
RUN pip3 -q install pip --upgrade
RUN pip3 install jupyter
RUN python -m pip install ipykernel plotly iplot

# Make a build directory to work in
RUN mkdir -p /builds/src && mkdir -p /builds/build/master && mkdir -p /builds/plugins

#clone the master version of Sofa
RUN cd /builds/src && \
    git clone https://github.com/sofa-framework/sofa.git . && \
    git checkout 36e1030cdbffb1db458344be095b5d80f463b5c5

# clone specific version of STLIB, SoftRobots, and ModelOrderReduction
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

# build sofa
RUN cd /builds/build/master && \
    cmake -c CMakeCache.txt && \
    ninja -j 8
    
    
# Cleanup
RUN apt-get clean -y \
    && apt-get autoremove -y \
    && rm -rf /tmp/*

##############################################################################################
# Build End
##############################################################################################
# Project setup begin
##############################################################################################
# make the build directory accessible to anyone
RUN chmod -R a+wr /builds

# runSofa cannot be used by the root user so this must be added.
RUN useradd --create-home --shell /bin/bash sofauser && echo "sofauser:sofauser" | chpasswd && adduser sofauser sudo
WORKDIR /home/sofauser



# Set shell
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /run/user/1000 && chown sofauser:sofauser /run/user/1000/

USER sofauser
ENV HOME="/home/sofauser"

# set up environment with bashrc
RUN echo 'source /opt/qt512/bin/qt512-env.sh && exec "$@"' >> /home/sofauser/.bashrc
RUN echo 'export QTIFWDIR="/builds/Qt/Tools/QtInstallerFramework/3.0"' >> /home/sofauser/.bashrc
RUN echo 'export PYTHONPATH=/builds/plugins/ModelOrderReduction/python:/builds/src/tools/sofa-launcher:$PYTHONPATH' >> /home/sofauser/.bashrc 
RUN echo 'export PATH=/builds/build/master/bin:$PATH' >> /home/sofauser/.bashrc
RUN echo 'export PATH=$QTIFWDIR/bin:$PATH' >> /home/sofauser/.bashrc
RUN echo 'export XDG_RUNTIME_DIR=/run/user/1000' >> /home/sofauser/.bashrc



# Python2 kernel for jupyter notebook
RUN python -m ipykernel install --user

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD /bin/bash -c "source ~/.bashrc && cd /home/sofauser/workdir/MOR_test && /bin/bash"

