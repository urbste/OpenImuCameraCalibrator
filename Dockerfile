FROM ubuntu:22.04

ENV ceresVersion="2.1.0"
ENV pyTheiaVersion="69c3d37"
ENV NUM_PROC=5

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
    libdc1394-dev libopenexr-dev \
    libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev \
	libgoogle-glog-dev \
	libgflags-dev \
	libeigen3-dev \
	libopencv-dev \
	libopencv-contrib-dev

RUN git clone https://github.com/ceres-solver/ceres-solver && \
    cd ceres-solver && \
	git checkout $ceresVersion && \
	mkdir -p build && cd build && \
	cmake .. -DBUILD_EXAMPLES=OFF -DCMAKE_BUILD_TYPE=Release && \
	make -j${NUM_PROC} install && \
    cd ../../ && rm -r ceres-solver 

RUN git clone https://github.com/urbste/pyTheiaSfM && \
    cd pyTheiaSfM && \
	git checkout $pyTheiaVersion && \
	mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && \
	make -j${NUM_PROC} install && \
    cd ../../ && rm -r pyTheiaSfM

WORKDIR /

COPY . OpenImuCameraCalibrator

RUN cd OpenImuCameraCalibrator && \
	mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && \
	make -j${NUM_PROC} && cd .. && pip3 install -r requirements.txt

# create a symbolic link for python3 to python
RUN ln -s /usr/bin/python3 /usr/bin/python
