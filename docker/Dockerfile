FROM ros:noetic-perception

RUN apt-get update && apt-get install -y wget git gpg ca-certificates cmake build-essential

RUN git clone --depth 1 --single-branch --branch 4.2 https://github.com/borglab/gtsam.git && \
    cd gtsam && mkdir build && cd build && \
    cmake -DBUILD_EXAMPLES_ALWAYS=OFF -DBUILD_TESTS=OFF .. && make && make install

RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main' | tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt-get update && apt-get install -y --no-install-recommends --reinstall kitware-archive-keyring cmake

RUN git clone --depth 1 --single-branch --branch v0.17.0 https://github.com/isl-org/Open3D.git && \
    Open3D/util/install_deps_ubuntu.sh assume-yes && \
    cd Open3D && mkdir build && cd build && \
    cmake -DBUILD_EIGEN3=ON -DBUILD_GLEW=ON -DBUILD_GLFW=ON -DBUILD_JSONCPP=ON -DBUILD_PNG=ON -DGLIBCXX_USE_CXX11_ABI=ON -DBUILD_PYTHON_MODULE=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_EXAMPLES=OFF .. && \
    make && make install

WORKDIR /root/catkin_ws

COPY paloc src/paloc

RUN . /opt/ros/noetic/setup.sh && catkin_make

RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc


