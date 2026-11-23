# dp_fwd_inv_dynamics
Forward and Inverse Dynamics of a Double Pendulum

In this tutorial you derive and implement the forward and inverse dynamics of a
double pendulum, simulate it, and run gravity compensation and PD controllers
both in simulation and on a real double pendulum through
[CloudPendulum](https://github.com/cloudpendulum).

## Contents

| File | Description |
|------|-------------|
| `fwd_inv_dyn_student.ipynb` | Tutorial notebook with exercises (start here) |
| `fwd_inv_dyn_teacher.ipynb` | Same notebook with solutions |
| `double_pendulum/` | Plant model, simulator/animation and controllers used by the notebooks |
| `cloud_pendulum_local.py` | Client for running experiments on locally connected hardware |
| `robot.json` | Motor configuration for the local hardware client |

## Running on the CloudPendulum JupyterHub

1. Open a terminal in JupyterLab, clone the repo and move inside it:
   ```
   git clone https://github.com/cloudpendulum/dp_fwd_inv_dynamics.git
   cd dp_fwd_inv_dynamics
   ```
2. Open `fwd_inv_dyn_student.ipynb` and run the first cell to install the
   dependencies from `requirements.txt`.
3. To run experiments on the real pendulum, get your user token from
   *File → Hub Control Panel → CP Token & Compute Status* and paste it into the
   `user_token` cell of the notebook. Do not commit or share your token.

The hub environment provides the CloudPendulum client (`pycloudpendulum`) and
`ffmpeg`, which is needed to convert the experiment recordings to MP4.

### Notes

- Simulations are computed with the step size `dt`, while the animation only
  draws one frame every `anim_dt` seconds (default 0.04 s = 25 fps). Increase
  `anim_dt` if rendering the animations is too slow.
- After an experiment, the recording is downloaded into the repo folder
  (`*.flv`/`*.mp4`). The server may need a short while to finalize it; if it is
  not available within two minutes, the notebook prints a link to it instead.

## Running locally with Docker (local hardware)

To run the experiments on a double pendulum connected to your own computer, set
`user_token = None` in the notebook. Experiments then use
`cloud_pendulum_local.py`, which talks to the motors through `pyCandle` using
the motor IDs in `robot.json`.

1. Install docker on Ubuntu using the instructions
   [here](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)
   and follow the
   [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/).
2. Pull the docker image:
   ```
   docker pull costashatz/optrob25:latest
   ```
3. Clone the repo and move inside it:
   ```
   git clone https://github.com/cloudpendulum/dp_fwd_inv_dynamics.git
   cd dp_fwd_inv_dynamics
   ```
4. Start the container:
   ```
   docker run --rm -it --privileged -v /sys:/sys -v /dev:/dev -v /run/udev:/run/udev --net=host -e DISPLAY -v ${HOME}/.Xauthority:/home/robot/.Xauthority -v "$(pwd)":/home/robot/code --entrypoint /bin/bash costashatz/optrob25
   ```
5. Inside the container, start `jupyter lab` and open `fwd_inv_dyn_student.ipynb`.
