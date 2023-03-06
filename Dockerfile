FROM ubuntu:20.04

ENV cvVersion="4.5.1"
ENV ceresVersion="2.0.0"
ENV pyTheiaVersion="ca50599"
ENV NUM_PROC=20
ENV NODE_VERSION=14

RUN apt-get update && \
	apt-get remove -y && \
	DEBIAN_FRONTEND=noninteractive && \ 
    apt-get install -y tzdata && \ 
    TZ=Europe/Paris && \
    apt-get install -y \
	build-essential \
	checkinstall \
	cmake \
	pkg-config \
	git \
	gfortran \
    libgtk-3-dev \
    libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
    libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
    gfortran openexr libatlas-base-dev python3-dev python3-numpy python3-pip \
    libtbb2 libtbb-dev libdc1394-22-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
    libsuitesparse-dev \
	libgoogle-glog-dev \
	libgflags-dev \
	libeigen3-dev 

RUN curl -sL https://deb.nodesource.com/setup_$NODE_VERSION.x | bash - && \
    apt-get install -y nodejs


RUN git clone https://github.com/opencv/opencv.git && \
    git clone https://github.com/opencv/opencv_contrib.git && \
	cd opencv_contrib && \
	git checkout $cvVersion && \
    cd .. && \
    cd opencv && \
	git checkout $cvVersion && \
	mkdir -p build && \
	cd build && \
	cmake .. -DCMAKE_BUILD_TYPE=RELEASE \
	-DCMAKE_INSTALL_PREFIX=/usr/local \
	-DINSTALL_C_EXAMPLES=OFF \
	-DWITH_TBB=ON \
	-DWITH_V4L=ON \
	-DWITH_QT=OFF \
	-DWITH_OPENGL=OFF \
	-DWITH_EIGEN=ON \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_WITH_DEBUG_INFO=OFF \
    -DBUILD_LIST=aruco,core,videoio,video,calib3d,highgui \
	-DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules && \
	make -j${NUM_PROC} install && \
	cd ../../ && rm -r opencv && rm -r opencv_contrib

RUN git clone https://github.com/ceres-solver/ceres-solver && \
    cd ceres-solver && \
	git checkout $ceresVersion && \
	mkdir -p build && cd build && \
	cmake .. -DBUILD_EXAMPLES=OFF && \
	make -j${NUM_PROC} install && \
    cd ../../ && rm -r ceres-solver 

RUN git clone https://github.com/urbste/pyTheiaSfM && \
    cd pyTheiaSfM && \
	git checkout $pyTheiaVersion && \
	mkdir -p build && cd build && cmake .. && \
	make -j${NUM_PROC} install && \
    cd ../../ && rm -r pyTheiaSfM

RUN git clone https://github.com/urbste/OpenImuCameraCalibrator && \
    cd OpenImuCameraCalibrator && \
	mkdir -p build && cd build && cmake .. && \
	make -j${NUM_PROC} && cd .. && pip3 install -r requirements.txt && \
	cd javascript && npm install

# create a symbolic link for python3 to python
RUN ln -s /usr/bin/python3 /usr/bin/python
