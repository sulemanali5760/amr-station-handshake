FROM ros:jazzy-ros-base

# git and curl for Codespaces; asyncua is not packaged for rosdep. --ignore-installed keeps
# pip from trying to upgrade apt-owned packages it cannot uninstall.
RUN apt-get update \
    && apt-get install -y --no-install-recommends git curl python3-pip \
    && pip install --break-system-packages --ignore-installed asyncua==2.0.1 \
    && rm -rf /var/lib/apt/lists/*

RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc \
    && echo '[ -f /workspaces/amr-station-handshake/install/setup.bash ] && source /workspaces/amr-station-handshake/install/setup.bash' >> /root/.bashrc
