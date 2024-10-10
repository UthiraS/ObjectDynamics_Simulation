# University of Lincoln - Precursor to Autonomous Manipulation

## Objective
The primary goal of this project is to understand the physics of a simulation environment in which a robot repeatedly builds and deconstructs a tower of blocks. The task involves using machine learning techniques to predict subsequent sequences of block positions and orientations after each manipulation action. The focus is on training a robot to interact intelligently with objects in its environment, pushing the boundaries of robot manipulation.

## Abstract

Robot manipulation is a critical area within robotics, particularly for tasks involving interaction with the physical world. This project emphasizes core manipulation tasks like **picking** and **placing objects** while intelligently interacting with the environment. It explores how machine learning algorithms can enhance robot decision-making, enabling a robot arm to autonomously choose the best actions to perform tasks.

Key objectives include:
- Manipulating a robot to execute **push** and **grasp actions** on blocks of different colors.
- Creating a **dataset of RGB images** that capture the positions and orientations of blocks during tower building and deconstruction.
- Utilizing the dataset to **predict future block positions** using a **machine learning architecture** based on **long short-term memory (LSTM) networks** combined with **convolutional encoder-decoder layers**.

This project uses simulation environments such as **V-REP**, **Gazebo**, and real-world robots like the **UR5** and **Franka Emika Panda** with **Intel RealSense** cameras to conduct the experiments and gather the necessary data.

## Keywords

- **Robot manipulation**
- **Machine learning**
- **Neural networks**
- **Degrees of freedom**
- **Sequence prediction**
- **Long short-term memory network (LSTM)**
- **Convolutional Encoder-Decoder**

## Project Overview

### 1. **Simulation and Environment Setup**
   - Environments used: V-REP, Gazebo.
   - Robots used: UR5, Franka Emika Panda.
   - Sensors: Intel RealSense Cameras for RGB image capture.

### 2. **Task Breakdown**
   - **Tower Construction and Deconstruction**: The robot builds a tower of blocks, then deconstructs it using push and grasp actions. 
   - **Data Collection**: RGB images of block positions and orientations are captured as the tower is manipulated.
   - **Data Annotation**: Each image is labeled with the corresponding block position and orientation.

### 3. **Machine Learning Model**
   - **Sequence Prediction**: LSTM-based architecture is employed to predict block positions in subsequent sequences.
   - **Network Architecture**:
     - **Encoder**: Convolutional layers down-sample RGB image data.
     - **LSTM**: Used to handle sequential data and predict future block states.
     - **Decoder**: Convolutional layers up-sample the data to reconstruct predictions of block positions and orientations.

### 4. **Evaluation**
   - The model's ability to predict the correct sequence of block movements is evaluated based on accuracy and consistency of the predicted positions compared to the actual results during the robot’s manipulation.

---

## Project File Structure

Precursor-to-Autonomous-Manipulation/
├── VREP FILES/
│   └── vrep-Franka/
├── Panda/meshes/
├── objects/
├── simulation/
│   └── test-cases/
│       ├── Franka-Panda.ttt
│       ├── _init_.py
│       ├── _init_.pyc
│       ├── remoteApi.so
│       ├── simulation.ttt
│       ├── vrep.py
│       ├── vrep.pyc
│       ├── vrepConst.py
│       ├── vrepConst.pyc
│       ├── Franka-Panda.ttt
│       ├── demo.py
│       ├── franka_emika.ttm
│       ├── panda.urdf
│       ├── transformations.py
│       ├── transformations.pyc
│       ├── utils.py
│       └── utils.pyc
├── vrep-URS/
│   ├── objects/
│   └── simulation/
│       ├── demo.py
│       ├── tower.py
│       ├── transformations.py
│       ├── ur5.py
│       ├── utils.py
│       └── utils.pyc
├── vrep/
│   └── Franka_Grasping.ttt
├── franka_active_sensing/
│   ├── arm_control/
│   ├── franka_arm_gripper_realsense_gazebo/
│   ├── launch/
│   ├── model/
│   ├── props_models/
│   ├── scripts/
│   │   ├── interim/
│   │   └── results/
│   │       ├── demo_compressed_img_subscriber.py
│   │       ├── demo_depth_subscriber.py
│   │       ├── demo_depth_subscriber_non_blocking.py
│   │       ├── demo_image_subscriber.py
│   │       ├── demo_image_subscriber.pyc
│   │       ├── depth_subscriber_segmentator.py
│   │       ├── depth_subscriber_segmentator_non_blocking.py
│   │       ├── get_gt_pcl.py
│   │       ├── init_models.py
│   │       └── pause_gazebo.py
│   ├── worlds/
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── ros_gazebo_moveit_setup.odt
├── towervideos/
└── Certificates.pdf
## 1. Franka Grasping

Demonstration of the Franka Emika Panda robot performing a grasping task using autonomous manipulation strategies.

[![Franka Grasping](path_to_thumbnail_image)](https://github.com/UthiraS/Precursor-to-Autonomous-Manipulation/blob/main/Franka_Grasping%20.mp4))  
*Click to watch the video.*

---

## 2. Franka Pushing

A video showing the Franka Emika Panda executing a pushing task, exploring active sensing and feedback-driven manipulation.

[![Franka Pushing](path_to_thumbnail_image)](link_to_Franka_pushing.mp4)  
*Click to watch the video.*

---

## 3. UR5 Manipulation Demo

This video highlights the UR5 robot demonstrating manipulation techniques with task-based objectives.

[![UR5 Manipulation](path_to_thumbnail_image)](link_to_UR5.mp4)  
*Click to watch the video.*

---

## 4. Push Demo

A push-based manipulation demo where the robot performs a series of movements to adjust objects in its environment.

[![Push Demo](path_to_thumbnail_image)](link_to_push_demo.mp4)  
*Click to watch the video.*

---

## 5. Active Sensing with Franka

Exploration of active sensing capabilities integrated into the Franka Emika Panda system.

[![Active Sensing](path_to_thumbnail_image)](link_to_franka_active_sensing.mp4)  
*Click to watch the video.*

---

## 6. Tower Building Demo

This video demonstrates an autonomous tower-building task, showcasing the precision and adaptive control of the robotic system.

[![Tower Building](path_to_thumbnail_image)](link_to_tower_videos.mp4)  
*Click to watch the video.*

---

For further details, refer to the [SAP Project Thesis](link_to_SAP_PROJECT_THESIS.pdf) which outlines the theoretical background, methodology, and results of the project.
