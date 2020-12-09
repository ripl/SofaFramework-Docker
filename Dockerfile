FROM ubuntu:18.04

# code adapted from 


SHELL ["/bin/bash", "-c"]

#ADD "https://www.sofa-framework.org/rand/" ForceNoCache

RUN apt-get update && apt-get upgrade -y

# Install tools
RUN apt-get install -y \
    apt-utils \
    git \
    net-tools \
    vim \
    wget \
    curl \
    zip \
    unzip

# Install compilers
RUN apt-get install -y \
    build-essential \
    software-properties-common \
    ninja-build \
    gcc-7 g++-7 \
    clang \
    ccache

# Install core deps
RUN apt-get install -y \
    libglew-dev \
    freeglut3-dev \
    zlib1g-dev \
    libeigen3-dev

# Install plugins deps
RUN apt-get install -y \
    python2.7-dev python-pip python-numpy python-scipy \
    libpng-dev libjpeg-dev libtiff-dev \
    libblas-dev \
    liblapack-dev \
    libsuitesparse-dev \
    libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
    libassimp-dev \
    libbullet-dev \
    liboce-ocaf-dev \
    libzmq3-dev liboscpack-dev
ENV VM_HAS_ASSIMP="true"
 # BulletColisionDetection is broken
ENV VM_HAS_BULLET="disabled"
ENV VM_HAS_OPENCASCADE="true"

# Install CMake
ADD https://github.com/Kitware/CMake/releases/download/v3.12.0/cmake-3.12.0-Linux-x86_64.sh /tmp
RUN chmod a+x /tmp/cmake-3.12.0-Linux-x86_64.sh
RUN /tmp/cmake-3.12.0-Linux-x86_64.sh --skip-license --prefix=/usr/local


# Install Qt
RUN add-apt-repository -y ppa:beineri/opt-qt-5.12.6-bionic \
    && apt-get update \
    && apt-get install -y qt512-meta-full qt512charts-no-lgpl
ADD https://www.sofa-framework.org/wp-content/uploads/2020/06/QtInstallerFramework_3.0_Linux.zip /tmp
RUN unzip /tmp/QtInstallerFramework_3.0_Linux.zip -d /builds && chmod -R a+x /builds/Qt
ENV QTIFWDIR="/builds/Qt/Tools/QtInstallerFramework/3.0"
ENV PATH="${QTIFWDIR}/bin:${PATH}"

# Install Boost
RUN add-apt-repository -y ppa:mhier/libboost-latest \
    && apt-get update \
    && apt-get install -y libboost1.67-dev

# Install CGAL
# Due to dependencies on Boost and Qt, we have to build CGAL
ADD https://github.com/CGAL/cgal/releases/download/releases/CGAL-4.14.3/CGAL-4.14.3.tar.xz /tmp
RUN apt-get install -y libgmp-dev libmpfr-dev
RUN tar -xJf /tmp/CGAL-4.14.3.tar.xz --directory /tmp \
    && cd /tmp/CGAL-4.14.3 \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DWITH_CGAL_Core=TRUE -DWITH_CGAL_ImageIO=TRUE -DWITH_CGAL_Qt5=TRUE .. \
    && make install
ENV VM_HAS_CGAL="true"
ENV VM_CGAL_PATH="/usr/local/lib/cmake/CGAL"

# Install CUDA
RUN apt-get install -y nvidia-cuda-toolkit
ENV VM_HAS_CUDA="true"
ENV VM_CUDA_HOST_COMPILER="/usr/bin/gcc-6"
ENV VM_CUDA_ARCH="sm_50"

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
RUN echo 'export source /opt/qt512/bin/qt512-env.sh && exec "$@"' >> /home/sofauser/.bashrc
RUN echo 'export QTIFWDIR="/builds/Qt/Tools/QtInstallerFramework/3.0"' >> /home/sofauser/.bashrc
RUN echo 'export PYTHONPATH=/builds/plugins/ModelOrderReduction/python:/builds/src/tools/sofa-launcher:$PYTHONPATH' >> /home/sofauser/.bashrc 
RUN echo 'export PATH=/builds/build/master/bin:$PATH' >> /home/sofauser/.bashrc
RUN echo 'export PATH=$QTIFWDIR/bin:$PATH' >> /home/sofauser/.bashrc
RUN echo 'export XDG_RUNTIME_DIR=/run/user/1000' >> /home/sofauser/.bashrc



# Python2 kernel for jupyter notebook
RUN python -m ipykernel install --user

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD /bin/bash -c "source ~/.bashrc && cd /home/sofauser/workdir/MOR_test && /bin/bash"

