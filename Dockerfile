FROM osrf/ros:jazzy-desktop-full

# Evitar prompts interactivos
ENV DEBIAN_FRONTEND=noninteractive

# 1. Herramientas básicas y dependencias de sistema para Webots
RUN apt-get update && apt-get install -y \
    nano \
    wget \
    curl \
    gnupg2 \
    ca-certificates \
    # Dependencias de control y navegación (Sin Gazebo)
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-twist-mux \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-ackermann-msgs \
    # RTAB-Map y dependencias de SLAM
    ros-jazzy-rtabmap-ros \
    # Dependencias gráficas y de audio necesarias para la UI de Webots
    libnss3 \
    libxcomposite1 \
    libxtst6 \
    libxrandr2 \
    libgbm1 \
    libxkbcommon-x11-0 \
    libpci-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 2. Instalar el repositorio oficial de Cyberbotics y Webots
RUN wget -qO- https://cyberbotics.com/Cyberbotics.asc | gpg --dearmor > /usr/share/keyrings/cyberbotics.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/cyberbotics.gpg] https://cyberbotics.com/debian/ binary-amd64/" > /etc/apt/sources.list.d/cyberbotics.list && \
    apt-get update && apt-get install -y \
    webots \
    ros-jazzy-webots-ros2 \
    && rm -rf /var/lib/apt/lists/*

# Definir variables de entorno esenciales para Webots
ENV WEBOTS_HOME=/usr/local/webots
ENV PATH=$WEBOTS_HOME:$PATH

# 3. Instalación de paquetes Python (Cuidado con --break-system-packages en Ubuntu 24.04)
RUN pip install --no-cache-dir --break-system-packages \
    numpy \
    scipy \
    catkin_pkg \
    matplotlib \
    torch \
    torchvision

# 4. Configuración de Usuario No-Root
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ENV USER=$USERNAME
ENV USERNAME=$USERNAME

RUN apt-get update && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && mkdir -p /home/$USERNAME/.config && chown -R $USER_UID:$USER_GID /home/$USERNAME \
    && usermod -aG video,render $USERNAME \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME

# Configuración del entorno en .bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /home/$USERNAME/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=0" >> /home/$USERNAME/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc && \
    echo "alias inst_r='source install/setup.bash'" >> /home/$USERNAME/.bashrc && \
    echo "export WEBOTS_HOME=/usr/local/webots" >> /home/$USERNAME/.bashrc

# Volver a root para manejar el entrypoint
USER root
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]