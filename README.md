# Doly ROS2 Interface

A Dockerized ROS2 interface for the [Doly robot](https://www.doly.ai/), built with the [C++ Doly SDK](https://github.com/robotdoly/DOLY-DIY/tree/main/SDK).

Docker: https://hub.docker.com/r/mcfurry/doly-ros-rolling

## Overview

This repository provides a ROS2 wrapper around the Doly SDK, enabling seamless integration of Doly robot into ROS2-based robotic systems. The Docker setup ensures minimal changes are needed on the OS of Doly itself.

## Features

- ROS2 integration with Doly robot control
- Dockerized environment for easy deployment
- Built on the official C++ Doly SDK

## Requirements

- Docker and Docker Compose

## Install docker on Doly

To install Docker on Doly, ssh into the system and run:

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
```

Then install Docker Compose:

```bash
sudo apt-get install docker-compose
```

Reboot your system to apply group changes.

## Quick Start

Make sure to copy `docker-compose.yml` to the Doly robot first.

⚠️ **Important:** The Docker container requires privileged mode and root user access to communicate with the robot's hardware. Start the container with appropriate permissions and as root user!.

```bash
sudo su
docker-compose up
```

## Resources

- [Doly Official Website](https://www.doly.ai/)
- [Doly SDK Repository](https://github.com/robotdoly/DOLY-DIY/tree/main/SDK)
- [ROS2 Documentation](https://docs.ros.org/)

## License

See LICENSE file for details.