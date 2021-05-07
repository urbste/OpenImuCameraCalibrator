FROM ubuntu:20.04

ENV cvVersion="4.5.1"
ENV ceresVersion="2.0.0"

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
	libgoogle-glog-dev \
	libgflags-dev \
	libeigen3-dev \
	curl gnupg ca-certificates && \
	curl -L https://deb.nodesource.com/setup_12.x | bash \
    apt-get update -yq && \
    apt-get install -yq && \
	dh-autoreconf=19 \
	ruby=1:2.5.* \
	ruby-dev=1:2.5.* \
	nodejs


RUN git clone https://github.com/opencv/opencv.git && \
    git clone https://github.com/opencv/opencv_contrib.git && \
	cd opencv_contrib && \
	git checkout $cvVersion && \
    cd .. && \
    cd opencv && \
	git checkout $cvVersion && \
	mkdir -p build && \
	cd build && \
	cmake -DCMAKE_BUILD_TYPE=RELEASE \
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
    -DBUILD_alphamat=OFF \
    -DBUILD_apps=OFF \
    -DBUILD_bgsegm=OFF \
    -DBUILD_bioinspired=OFF \
    -DBUILD_datasets=OFF\
    -DBUILD_dnn=OFF \
    -DBUILD_dnn_objdetect=OFF \
    -DBUILD_dnn_superres=OFF \
    -DBUILD_dpm=OFF \
    -DBUILD_face=OFF \
    -DBUILD_gapi=OFF \
    -DBUILD_hfs=OFF \
    -DBUILD_intensity_transform=OFF \
    -DBUILD_ml=OFF \
    -DBUILD_objdetect=OFF \
    -DBUILD_optflow=OFF \
    -DBUILD_phase_unwrapping=OFF \
    -DBUILD_photo=OFF \
    -DBUILD_rapid=OFF \
    -DBUILD_reg=OFF \
    -DBUILD_rgbd=OFF \
    -DBUILD_saliency=OFF \
    -DBUILD_sfm=OFF \
    -DBUILD_shape=OFF \
    -DBUILD_stereo=OFF \
    -DBUILD_stitching=OFF \
    -DBUILD_structured_light=OFF \
    -DBUILD_superres=OFF \
    -DBUILD_surface_match=OFF \
    -DBUILD_text=OFF \
    -DBUILD_tracking=OFF \
    -DBUILD_ts=OFF \
    -DBUILD_xphoto=OFF \ 
    -DBUILD_xobjdetect=OFF \
	-DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules && \
	make -j6 && make install && \
	cd .. && rm -r opencv && rm -r opencv_contrib

RUN git clone https://github.com/ceres-solver/ceres-solver && \
    cd ceres-solver && \
	git checkout $ceresVersion && \
	mkdir -p build && cd build \
	cmake .. -DBUILD_EXAMPLES=OFF && \
	make -j6 && make install


