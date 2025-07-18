# dp_fwd_inv_dynamics
Forward and Inverse Dynamics of a Double Pendulum

# Instructions (assuming docker is installed already):

1. Pull the docker image: 
```docker pull costashatz/optrob25:latest```

2. Pull the dp_fwd_inv_dynamics repo: 
```git clone https://github.com/cloudpendulum/dp_fwd_inv_dynamics.git``` 

3. Run the command:
```docker run --rm -it --privileged -v /sys:/sys -v /dev:/dev -v /run/udev:/run/udev --net=host -e DISPLAY -v ${HOME}/.Xauthority:/home/robot/.Xauthority -v "$(pwd)":/home/robot/code --entrypoint /bin/bash costashatz/optrob25```⁠

4. Run the notebook ```fwd_inv_dynamics_student.ipynb``` inside your docker.

# Docker Installation

* Install docker on your Ubuntu using the instructions here: https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository 
* Follow the post installation steps on Linux here: https://docs.docker.com/engine/install/linux-postinstall/ 
