FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ="Europe/Rome"

RUN apt update
RUN apt install xfce4 xfce4-goodies -y
RUN apt install xrdp -y
RUN apt install -y nano gedit net-tools
RUN apt install -y sudo iptables git python3-pip

RUN useradd -m -p $(openssl passwd -1 mivia) -s /bin/bash mivia
RUN usermod -aG sudo mivia
RUN adduser xrdp ssl-cert 

USER mivia
WORKDIR /home/mivia/
ADD run_xrdp.sh .
RUN echo "xfce4-session" | tee .xsession

RUN git clone https://github.com/gdesimone97/turtlebot4.git
WORKDIR turtlebot4/scripts

USER root

#Configure Timezone
RUN apt-get update && \
    apt-get install -yq tzdata && \
    ln -fs /usr/share/zoneinfo/Europe/Rome /etc/localtime && \
    dpkg-reconfigure -f noninteractive tzdata

RUN bash install_locales.bash
RUN bash ros2_humble_install.bash
RUN bash ros2-tools.bash
RUN bash turtlebot4_tools.bash

RUN update-alternatives --set x-terminal-emulator /usr/bin/xfce4-terminal.wrapper

USER mivia

WORKDIR /home/mivia
RUN echo "source /opt/ros/humble/setup.bash" >> .bashrc
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> .bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> .bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> .bashrc
CMD ["/bin/bash"]