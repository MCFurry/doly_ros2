# Stage 1: Build using ros-rolling-base
FROM ros:rolling-ros-base AS builder

COPY src /ws/src
COPY colcon_defaults.yaml /ws/colcon_defaults.yaml
WORKDIR /ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-rolling-rmw-zenoh-cpp \
    ros-rolling-ros2-fmt-logger

# Build and install spdlog 1.17.0 from source
RUN git clone --branch v1.17.0 --depth 1 https://github.com/gabime/spdlog.git /tmp/spdlog && \
    cd /tmp/spdlog && \
    cmake -B build -DCMAKE_BUILD_TYPE=Release -DSPDLOG_BUILD_SHARED=ON -DSPDLOG_BUILD_EXAMPLES=OFF -DSPDLOG_BUILD_TESTS=OFF && \
    cmake --build build --target install && \
    rm -rf /tmp/spdlog

RUN /ros_entrypoint.sh colcon build

# Stage 2: Minimal runtime image
FROM ros:rolling-ros-core

COPY --from=builder /ws/install /opt/doly/rolling
# Also take latest build spdlog
COPY --from=builder /usr/local/lib/libspdlog.so* /usr/local/lib/
RUN ldconfig

# Install zenoh RMW and cleanup apt cache to reduce image size
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-rolling-rmw-zenoh-cpp \
    ros-rolling-ros2-fmt-logger \
     && rm -rf /var/lib/apt/lists/*

# Update bashrc such that the ros-environment is sourced when someone enters the container
RUN echo 'source /opt/doly/rolling/setup.bash' >> ~/.bashrc

COPY start.bash /start.bash
RUN chmod +x /start.bash
CMD ["/start.bash"]
