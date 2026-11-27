FROM autonomymind/ros-gazebo-base:dev
RUN apt-get update && apt-get install -y --no-install-recommends ros-jazzy-ros-gz && rm -rf /var/lib/apt/lists/*
COPY simulation/gazebo /gazebo
COPY src/ /opt/autonomymind/src/
ENV PYTHONPATH=/opt/autonomymind/src
