# Stage 1: Build using ros-rolling-base
FROM ros:rolling-ros-base AS builder

COPY src /ws/src
COPY colcon_defaults.yaml /ws/colcon_defaults.yaml
WORKDIR /ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    libasound2-dev \
    ros-rolling-rmw-zenoh-cpp \
    ros-rolling-ros2-fmt-logger

# Build and install spdlog 1.17.0 from source
RUN git clone --branch v1.17.0 --depth 1 https://github.com/gabime/spdlog.git /tmp/spdlog && \
    cd /tmp/spdlog && \
    cmake -B build -DCMAKE_BUILD_TYPE=Release -DSPDLOG_BUILD_SHARED=ON -DSPDLOG_BUILD_EXAMPLES=OFF -DSPDLOG_BUILD_TESTS=OFF && \
    cmake --build build --target install && \
    rm -rf /tmp/spdlog

# Build libpiper runtime libs (includes onnxruntime download during cmake configure)
RUN git clone --depth 1 https://github.com/OHF-Voice/piper1-gpl.git /tmp/piper1-gpl && \
    cd /tmp/piper1-gpl/libpiper && \
    cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/piper && \
    cmake --build build && \
    cmake --install build && \
    rm -rf /tmp/piper1-gpl

RUN /ros_entrypoint.sh colcon build

# Stage 2: Minimal runtime image
FROM ros:rolling-ros-core

COPY --from=builder /ws/install /opt/doly/rolling
# Also take latest build spdlog
COPY --from=builder /usr/local/lib/libspdlog.so* /usr/local/lib/
COPY --from=builder /opt/piper /opt/piper
RUN ldconfig

# Install zenoh RMW and cleanup apt cache to reduce image size
RUN apt-get update && apt-get install -y --no-install-recommends \
    libasound2-dev \
    libespeak-ng1 \
    ros-rolling-rmw-zenoh-cpp \
    ros-rolling-ros2-fmt-logger \
     && rm -rf /var/lib/apt/lists/*

# Install piper/onnx runtime libs into dynamic linker path
RUN cp -a /opt/piper/lib/libonnxruntime.so* /usr/local/lib/ && \
    cp -a /opt/piper/libpiper.so* /usr/local/lib/ && \
    ln -sf /usr/local/lib/libpiper.so /usr/local/lib/libpiper_phonemize.so.1 && \
    ldconfig

# Update bashrc such that the ros-environment is sourced when someone enters the container
RUN echo 'source /opt/doly/rolling/setup.bash' >> ~/.bashrc

COPY start.bash /start.bash
RUN chmod +x /start.bash
CMD ["/start.bash"]
