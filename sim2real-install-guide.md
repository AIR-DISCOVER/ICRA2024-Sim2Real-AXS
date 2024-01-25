# Installing the system

## Software Architecture - Docker

Docker is an open-source containerization platform that allows developers to package an application and its dependencies into a single container. This container contains everything needed to run the application in any Docker-enabled environment, thus ensuring consistency, portability, and repeatability. 

Docker has a wide range of uses in various application scenarios:

* Application Development and Testing: Developers can use Docker containers to build and test applications in local development environments, ensuring that they work across different environments. This helps solve the "it works on my machine" problem.
* Continuous Integration/Continuous Deployment (CI/CD): Docker containers are integrated with CI/CD tools to automate the building, testing, and deploying of applications, improving the speed and quality of delivery.
* Multi-Cloud and Hybrid Cloud Deployment: Docker containers can run in different cloud providers' environments, enabling multi-cloud and hybrid cloud deployment strategies that provide flexibility and elasticity.
*  Microservice Architecture: Each microservice can be packaged as a separate Docker container, making the microservice architecture easier to manage, scale, and deploy. Container orchestration tools such as Kubernetes can be used to automate the management of microservices.
*  Rapid Scaling and Load Balancing: Docker containers can scale rapidly to meet changing load demands, and container orchestration tools can automate load balancing to ensure high availability and performance.
* Failure Isolation and Security: Each Docker container is isolated and does not interfere with the operations of other containers, helping to improve application security and availability.
* Data Processing and Analytics: Docker containers can be used to package and deploy data processing tasks, accelerating data analytics workloads while providing portability and scalability.

Overall, Docker provides a modernized way to build, deploy, and manage applications, helping to streamline development and operations processes, and improving application reliability and maintainability.
Given these capabilities, Docker has a wide range of uses in various application scenarios and is an important part of containerization technology.

## Host operation

### 1.1 Docker

If docker local installation hasn't been done, switch to the docker_server folder first:

```
cd ICRA2024-Sim2Real-AXS
```

Execution:

```
./scripts/docker_install.sh 
```
 

Evaluation

```
docker --version
```
example:

<div align="center">
  <img src="./assets/docker_version.png" width="80%">
</div>


Reference for docker installation on Ubuntu:

* [docker install](https://docs.docker.com/engine/install/ubuntu/)

**If the shell script cannot be run, check if there is permission for the script. Otherwise change the mode with chmod**


## 1.2 Nvidia driver

Check the version of host GPU driver before creating the docker and container, carefully keeping the same with the version inside docker. Currently the NVIDIA driver version inside the docker repos is 470.86.

In Ubuntu, ```Software & Updates > Additional Drivers``` is recommanded to update the Nvidia driver.

Open the terminal, input nvidia-smi and press enter to get the driver version:

<div align="center">
  <img src="./assets/nvidia_smi.png" width="80%">
</div>



Know issue: If your OS is Ubuntu21.04 or later, please refer [issue](https://codeantenna.com/a/nRJ1FuHehu) to fix

## 1.3 Install the nvidia-docker2

### 1.3.1 Main stages for docker installation reference

```
sudo systemctl --now enable docker

distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   && curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
   && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker

# test
sudo docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```
<div align="center">
  <img src="./assets/nvidia_docker.png" width="80%">
</div>


Reference link for docker installation: [docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)


## 1.4 Docker login

Register the dockerhub account:

* [dockerhub](https://hub.docker.com/)

And get access token for late use:

* https://docs.docker.com/docker-hub/access-tokens/

     
login the docker account:

```
sudo docker login
```
<div align="center">
  <img src="./assets/docker_login.png" width="80%">
</div>


## 1.5 Docker Server operation

### 1.5.1 Create container

Use scripts to create docker container

After clone this repo, scripts need to have execute right. Run code below to add rights: 

```
cd ICRA2024-Sim2Real-AXS
sudo chmod a+x scripts/*
```

To create container for OmniGibson environment, use run_omni.sh

```
./scripts/run_omni.sh
```

To create container for baseline, use run_baseline.sh

```
./scripts/run_baseline.sh
```

If the image have not found locally, it will pull from docker hub automatically. The image size for OmniGibson is about 60 Gb, for baseline is about 30 Gb. It will take some time to download. 

**Please check the image tag in the scripts.** Current docker images are jieyitsinghuawx/icra2024-sim2real-axs-env:v1.0.1 and jieyitsinghuawx/icra2024-sim2real-axs-baseline:v1.0.0. To get newest tag, please follow the challenge website. 

### 1.5.2 Execute container

The container can also be started and execute by using scripte. 

To execute container for OmniGibson environment, use exec_env.sh
```
./scripts/exec_env.sh
```

To execute container for baseline, use exec_baseline.sh
```
./scripts/exec_baseline.sh
```

### <a name="gibson"></a>1.5.3 OmniGibson environment

This docker container is the environment for the challenge. It is not allowed to be change. Any change in this container will not be accepted in submission. 

Caution: Every step below need to run in OmniGibson environment container. Make sure commands run in the container. 

1. Start the OmniGibson simulator

    Start a new terminal and execute the OmniGibson environment container. The conda environment should be `omnigibson`. If not, run:
    ```
    conda activate omnigibson
    ```
    After make sure the conda environment, you can start the simulator by running command below:

    ```
    roscore &
    python -m omnigibson.AXS_env --ik
    ```

2. Start ros TF publish

    Start a new terminal and execute the OmniGibson environment container. 
    
    To start TF publish, run: 
    ```
    roslaunch airbot_play_launch robot_state_publisher.launch robot_description_path:=/root/OmniGibson-Airbot/omnigibson/data/assets/models/airbot_play_with_rm2/airbot_with_texture/urdf_obj/AIRBOT_V3_v2-3.urdf &
    roslaunch airbot_play_launch static_transform_publisher.launch &
    ```

    These two program will run at the backend. If you do not want it at backend, please delete `&` at the end of command. If you do this, you will need two terminals which inside container. 

3. Start IK service

    If you run last step in backend, then you can continue work with the same terminal. Otherwise, start a new terminal and execute the OmniGibson environment container. 

    Run command below to start IK service:
    ```
    roslaunch airbot_play_launch airbot_play_moveit.launch use_rviz:=true target_moveit_config:=airbot_play_v2_1_config use_basic:=true
    ```

4. (**Optional**) Start ros keyboard control

    Start a new terminal and execute the OmniGibson environment container.

    Run command below to start keyboard control:
    ```
    python /root/OmniGibson-Airbot/teleop_twist_keyboard_AXS.py
    ```

    Usage: 

    Movement:

    `i` / `,`: Move forward / backward

    `j` / `l`: Rotate left / right in place

    `u` / `o`: Move in arc (front-left / front-right)

    `m` / `.`: Move in arc (back-left / back-right)

    `k`: Stop

    `q` / `z`: Increase / Decrease both linear and angular speed by 1.1 times

    `w` / `x`: Increase / Decrease linear speed by 1.1 times

    `e` / `c`: Increase / Decrease angular speed by 1.1 times

    Arm (joint control):

    `1` / `2`: Rotate arm joint 1

    `3` / `4`: Rotate arm joint 2

    `5` / `6`: Rotate arm joint 3

    `7` / `8`: Rotate arm joint 4

    `9` / `0`: Rotate arm joint 5

    `-` / `=`: Rotate arm joint 6

    Gripper:

    `a`: Open / Close gripper

### 1.5.4 Baseline

This docker contains the baseline for this challenge. The solution also suggest to complete inside this container. The submission will accept a docker image which contains solutions. 

To start baseline, please finish all steps in [1.5.3 OmniGibson environment](#gibson) and then follow the step below. 

1. Start hdl localization

    Start a new terminal and execute the baseline container. Run 
    ```
    roslaunch hdl_localization hdl_localization.launch
    ```
    and check the localization is same as current pose in simulator. If not, use `2D Pose Estimate` to correct it. 

2. Start base control

    Start a new terminal and execute the baseline container. The conda environment should be `baseline`. If not, run:
    ```
    conda activate baseline
    ```
    After make sure the conda environment, you can start the base control by running command below:
    ```
    python /root/robot_tools/examples/ros_base_control.py
    ```
  
3. Start main baseline service

    Start a new terminal and execute the baseline container. The conda environment should be `baseline`. If not, run:
    ```
    conda activate baseline
    ```
    After make sure the conda environment, you can start the basline program by running command below:
    ```
    python /root/Workspace/AXS_baseline/ICRA2024-Sim2Real-AXS/src/airbot/example/AXS_baseline.py
    ```
