FROM ros:iron-ros-base-jammy
ARG USERNAME=USERNAME
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid "$USER_GID" "$USERNAME" \
    && useradd --uid "$USER_UID" --gid "$USER_GID" -m "$USERNAME" \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo "$USERNAME" ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/"$USERNAME" \
    && chmod 0440 /etc/sudoers.d/"$USERNAME" \
    && apt-get clean

# Install dependencies
RUN apt update && apt upgrade -y \
    && apt install -y python3-pip \
    && apt install ros-"$ROS_DISTRO"-rviz2 -y \
    && apt install openssh-client -y \
    && apt install openssh-server -y \
    && apt install git -y \
    && apt install vim -y \
    && apt install python-is-python3 -y \
    && apt install clang-format -y \
    && apt install bear -y \
    && apt install curl -y \
    && curl -s https://deb.nodesource.com/setup_16.x | sudo bash \
    && apt install nodejs -y \
    && pip install black \
    && pip install rosbags \
    && pip install PyQT5 \
    && apt install ros-$ROS_DISTRO-xacro -y \
    && apt install ros-$ROS_DISTRO-foxglove-bridge -y \
    && apt clean

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/"$USERNAME"/.bashrc
EXPOSE 8765
ENV SHELL /bin/bash
USER $USERNAME
CMD ["/bin/bash"]