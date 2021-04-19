FROM nvidia/cudagl:10.1-devel-ubuntu18.04

RUN apt-get update --fix-missing && apt-get upgrade -y
# Install tools
RUN apt-get install -y \
    apt-utils \
    git \
    net-tools \
    vim \
    wget \
    curl \
    zip \
    unzip \
    patchelf \
    software-properties-common \
    subversion


# Install compilers
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
    apt-get update -y
RUN apt-get install -y \
    build-essential \
    ninja-build \
    gcc-9 g++-9 \
    clang \
    ccache


###############################################################################
### BLENDER install -- used for manipulating robot meshes
###############################################################################
RUN apt-get install -y \
  cmake \
  libx11-dev \
  libxrender-dev \
  libxxf86vm-dev \
  libxcursor-dev \
  libxi-dev \
  libxrandr-dev \
  libxinerama-dev \
  libglew-dev \
  libxft2 \
  libxft2:i386 \
  lib32ncurses5 \
  libxext6 \
  libxext6:i386
RUN mkdir -p /builds/blender-git && cd /builds/blender-git && \
 git clone https://git.blender.org/blender.git && \
 mkdir /builds/blender-git/lib && \
    cd /builds/blender-git/lib && \
    svn checkout https://svn.blender.org/svnroot/bf-blender/trunk/lib/linux_centos7_x86_64 && \
 update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 60 --slave /usr/bin/g++ g++ /usr/bin/g++-7 && \
 cd /builds/blender-git/blender && git checkout v2.91.2 && \
 cd /builds/blender-git/blender && make update -j 8 && \
 cd /builds/blender-git/blender && make bpy -j 8 && \
 cp -r /builds/blender-git/lib/linux_centos7_x86_64/python/lib/python3.7/site-packages /tmp && \
 rm -r /builds/blender-git && \
 mkdir -p /builds/blender-git/lib/linux_centos7_x86_64/python/lib/python3.7/site-packages && \
 mv /tmp/site-packages /builds/blender-git/lib/linux_centos7_x86_64/python/lib/python3.7

###############################################################################
###############################################################################


# Install core deps
RUN apt-get install -y \
    libglew-dev \
    freeglut3-dev \
    zlib1g-dev \
    libeigen3-dev

###############################################################################
# Install Sofa dependencies
###############################################################################
# Install plugins deps
RUN apt-get install -y \
    python2.7-dev python-pip \
    python3.7-dev python3-pip \
    libpng-dev libjpeg-dev libtiff-dev \
    libblas-dev \
    liblapack-dev \
    libsuitesparse-dev \
    libavcodec-dev libavformat-dev libavutil-dev libswscale-dev \
    libassimp-dev \
    libbullet-dev \
    liboce-ocaf-dev \
    libzmq3-dev liboscpack-dev
RUN python2.7 -m pip install numpy
RUN python3.7 -m pip install numpy
ENV VM_HAS_ASSIMP="true"
 # BulletColisionDetection is broken
ENV VM_HAS_BULLET="disabled"
ENV VM_HAS_OPENCASCADE="true"

# install mesh utils
RUN python3.7 -m pip install --user gmsh pygmsh meshio

###################################

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
    && mkdir -p build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DWITH_CGAL_Core=TRUE -DWITH_CGAL_ImageIO=TRUE -DWITH_CGAL_Qt5=TRUE .. \
    && make install
ENV VM_CGAL_PATH="/usr/local/lib/cmake/CGAL"

# Install pybind11 (needed by SofaPython3)
RUN git clone -b v2.4 --depth 1 https://github.com/pybind/pybind11.git /tmp/pybind11/src \
    && mkdir /tmp/pybind11/build && cd /tmp/pybind11/build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3.7 -DPYBIND11_TEST=OFF ../src \
    && make && make install

# Install NodeEditor (needed by SofaQtQuick)
RUN git clone -b 2.1.3 --depth 1 https://github.com/paceholder/nodeeditor.git /tmp/nodeeditor/src \
    && mkdir /tmp/nodeeditor/build && cd /tmp/nodeeditor/build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF ../src \
    && make && make install

###################################

# Disable core dumps
RUN printf '\n# Disable core dumps\n* soft core 0\n* hard core 0\n' >> /etc/security/limits.conf

# Init /builds directory
WORKDIR /builds

# Set env vars
ENV HOME="/root"
ENV PYTHONIOENCODING="UTF-8"
ENV PYTHONUSERBASE="/tmp/pythonuserbase"
RUN mkdir -p "$PYTHONUSERBASE" && chmod -R 777 "$PYTHONUSERBASE"
ENV PYTHON3_EXECUTABLE="/usr/bin/python3.7"

###############################################################################
###############################################################################



###############################################################################
### SOFA install
###############################################################################
RUN apt-get install -y ffmpeg

# Python2 Dependencies for Model Order Reduction
RUN apt-get install -y \
    python-qt4 \
    python-yaml \
    python-cheetah \
    python-nbformat

# Other python2 dependencies
RUN PIP_TARGET=/usr/lib/python2.7/dist-packages python2 -m pip install numpy

RUN mkdir -p /builds/src && mkdir -p /builds/build/master && mkdir -p /builds/plugins
COPY ./pluginsCMakeLists.txt /builds/plugins/CMakeLists.txt
COPY ./makefileCMakeCache.txt /builds/build/master/CMakeCache.txt

# clone specific version of Sofa, STLIB, SoftRobots, and ModelOrderReduction
RUN cd /builds/src && \
    git clone https://github.com/sofa-framework/sofa.git . && \
    git checkout 36e1030cdbffb1db458344be095b5d80f463b5c5 && \

    cd /builds/plugins && \
    git clone https://github.com/SofaDefrost/STLIB.git && \
    cd STLIB && \
    git checkout 6f329f61af7be3d2baab997e8318e95fde4fecca && \

    cd /builds/plugins && \
    git clone https://github.com/SofaDefrost/SoftRobots.git && \
    cd SoftRobots && \
    git checkout 93a7d0ed6658b0819cefbd4c91b90f5dae64be78 && \

    cd /builds/plugins && \
    git clone https://github.com/SofaDefrost/ModelOrderReduction.git && \
    cd ModelOrderReduction && \
    git checkout 83931e5697770e441e15dc3666c32f71fc038983 && \

    # Build
    cd /builds/build/master && \
    cmake -c CMakeCache.txt && \
    make -j 8 && \


    cd /builds/build/master && \
    make install && \
    mv /builds/build/master/install /builds/sofa && \
    mkdir /builds/python && \
    mv /builds/plugins/ModelOrderReduction/python/* /builds/python/ && \
    mv /builds/plugins/STLIB/python/* /builds/python/ && \
    mv /builds/plugins/SoftRobots/python/* /builds/python/ && \
    mv /builds/src/tools/sofa-launcher/* /builds/python/ && \
    rm -rf /builds/plugins /builds/src /builds/build

###############################################################################
###############################################################################


##############################################################################
# Install pytorch and dl deps
##############################################################################
RUN PIP_TARGET=/usr/lib/python3.7/dist-packages python3.7 -m pip install \
    torch==1.7.1+cu101 torchvision==0.8.2+cu101 -f https://download.pytorch.org/whl/torch_stable.html
RUN PIP_TARGET=/usr/lib/python3.7 python3.7 -m pip install --upgrade pip
RUN PIP_TARGET=/usr/lib/python3.7/dist-packages python3.7 -m pip install \
    gin-config \
    gym[atari] \
    gym[box2d] \
    imageio==2.9.0 \
    psutil \
    pyyaml \
    pandas \
    tensorboard

ENV LC_ALL="C.UTF-8"
ENV LANG="C.UTF-8"
RUN apt-get install -y nvidia-common && \
    DEBIAN_FRONTEND=noninteractive ubuntu-drivers autoinstall

# Install mesh dependencies

RUN cd /usr/local && \
    wget -nc  http://gmsh.info/bin/Linux/gmsh-4.6.0-Linux64-sdk.tgz && \
    tar -xf gmsh-4.6.0-Linux64-sdk.tgz && \
    rm gmsh-4.6.0-Linux64-sdk.tgz
ENV PYTHONPATH=/usr/local/gmsh-4.6.0-Linux64-sdk/lib:$PYTHONPATH
ENV PATH=/usr/local/gmsh-4.6.0-Linux64-sdk/bin:$PATH


##############################################################################
# Weird kernel issue with qt
##############################################################################
RUN strip --remove-section=.note.ABI-tag /opt/qt512/lib/libQt5Core.so.5

##############################################################################
# Add sofauser and cleanup
# runSofa cannot be used by the root user so this must be added.
##############################################################################

# install other tools
RUN apt-get update --fix-missing
RUN apt-get install -y tree x11-utils xserver-xorg-video-dummy
RUN PIP_TARGET=/usr/lib/python3.7/dist-packages python3.7 -m pip install \
    pynput pyautogui
RUN DEBIAN_FRONTEND=noninteractive  apt-get install -y python3-tk python3-dev
ENV DISPLAY :0
COPY dummy.conf /dummy.conf

# Cleanup
RUN apt-get clean -y \
    && apt-get autoremove -y \
    && rm -rf /tmp/*

# In-process env settings
COPY docker-entrypoint.sh /
RUN chmod a+x /docker-entrypoint.sh

ENV PATH="$PATH:/builds/sofa/bin"
RUN rm /usr/bin/python3 && ln -s python3.7 /usr/bin/python3
RUN rm /usr/bin/python && ln -s python3.7 /usr/bin/python
RUN mv /builds/blender-git/lib/linux_centos7_x86_64/python/lib/python3.7/site-packages/bpy.so /usr/lib/python3.7/dist-packages && \
    mv /builds/blender-git/lib/linux_centos7_x86_64/python/lib/python3.7/site-packages/2.91 /usr/lib/python3.7/dist-packages && \
    rm -r /builds/blender-git

ENV PYTHONPATH "/pkgs:/builds/python:$PYTHONPATH"

RUN apt-get install -y sudo
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
RUN adduser --disabled-password --gecos '' sofauser && adduser sofauser sudo
RUN chown -R sofauser:sofauser /builds
USER sofauser
WORKDIR /home/sofauser
ENV HOME="/home/sofauser"
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD /bin/bash -c "source ~/.bashrc && cd ~ && /bin/bash "
