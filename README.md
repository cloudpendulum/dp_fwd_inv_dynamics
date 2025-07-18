# dp_fwd_inv_dynamics
Forward and Inverse Dynamics of a Double Pendulum

# Instructions (assuming docker is installed already):

1. Pull the docker image: 
```docker pull costashatz/optrob25:latest```

2. Clone the dp_fwd_inv_dynamics repo and move inside it: 
```git clone https://github.com/cloudpendulum/dp_fwd_inv_dynamics.git``` 
```cd dp_fwd_inv_dynamics```

4. Run the command:
```docker run --rm -it --privileged -v /sys:/sys -v /dev:/dev -v /run/udev:/run/udev --net=host -e DISPLAY -v ${HOME}/.Xauthority:/home/robot/.Xauthority -v "$(pwd)":/home/robot/code --entrypoint /bin/bash costashatz/optrob25```⁠

5. Run the notebook ```fwd_inv_dynamics_student.ipynb``` inside your docker using the command ```jupyter lab```.

# Docker Installation

* Install docker on your Ubuntu using the instructions here: https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository 
* Follow the post installation steps on Linux here: https://docs.docker.com/engine/install/linux-postinstall/ 
