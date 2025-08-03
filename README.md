# Bootcamp for Fullstack Robotics Development

<table align="center">
    <tr>
      <td align="center">
        <img src="./media/fredo1_real.jpeg" alt="fredo1" width="300"/>
      </td>
      <td align="center">
        <img src="./media/fredo1_mujoco.png" alt="fredo1 twin" width="300"/>
      </td>
    </tr>
</table>

## Introduction
This is a learning bootcamp for fullstack robotics development. The term "fullstack" here mainly refers the process from robot conceptualizing to 3D modelling, to numerical simulation, to firmware embedding and to final controller design. Here, we 

- homemade a naive 3-DOF manipulator from scratch with different motors, 
- did simulations in mujoco/gazebo/bullet,
- maintained a digital twin with those platform,
- played with some fancy data-driven (PPO & Koopman) controllers.

Again, fundamentals in those aspects are grossly omitted; for those who have some basic knowledge, you are in the right place. In which, we have several components:

1. 3D modelling: 
    - [fredo1](./fredo1/cad)
    - [fredo2](./fredo2/cad)
2. Numerical Simulation Platforms
    - Gazebo + ROS1: 
      - [fredo1](./fredo1/gazebo) 
      - [dummy](./dummy/gazebo)
    - Mujoco: 
      - [fredo1](./fredo1/mujoco) 
      - [fredo2](./fredo2/mujoco) 
      - [dummy](./dummy/mujoco)
    - Pybullet: 
      - [fredo1](./fredo1/pybullet/) 
      - [dummy](./dummy/pybullet)
3. Embedding with Arduino: 
    - [fredo1](./fredo1/fredo1_firmware/): analog servo w/ feedback
    - [fredo2](./fredo2/fredo2_firmware/): digital velo servo w/ feedback
4. Digital Twinning (kinda):
    - Gazebo + ROS1: 
      - [fredo1](./fredo1/gazebo/) 
      - [dummy](./dummy/gazebo/)
    - Mujoco: 
      - [fredo1](./fredo1/mujoco/) 
      - [fredo2](./fredo2/mujoco/) 
      - [dummy](./dummy/mujoco/)
    - Pybullet: 
      - [fredo1](./fredo1/pybullet/) 
      - [dummy](./dummy/pybullet/)
5. Inverse Kinematics (w/ fredo1)
    - Homogeneous
    - D-H Parameters
6. Data-Driven Methods Demonstration:
    - RL w/ PPO
    - Koopman

## TO DO
- IK
- v-servo
- FOC control
- RL + DD

## Specs
- DoF: 3 (0~180 || >360)
- Computer: Lattepanda Alpha M3-7Y30 w/ 8 Gb RAM + any ubuntu 20.04 laptop
- Communication: UDP + serial tty
- Motors: servo SG-5010 (mod), servo Xinhui-velo (mod), FOC
- Material: Ledo6060 + PLA

## Usage
Within my implementation, this repo is cloned on my laptop, while [firmware1](./fredo1/fredo1_firmware/) & [firmware2](./fredo2/fredo2_firmware/) are on Lattepanda.

For setting up & launching, please refer to the following:
 - [Low-level Com](./fredo1/fredo1_firmware/) or [here](./fredo2/fredo2_firmware/)
 - [ROS + Gazebo](./fredo1/gazebo/)
 - [Mujoco](./fredo1/mujoco) or [here](./fredo2/mujoco/)
 - [Bullet](./fredo1/bullet)

## References
```
@misc{yasunori2024mujoco,
  author       = {Yasunori},
  title        = {MuJoCo model yourself},
  year         = 2024,
  note         = {\url{https://yasunori.jp/en/2024/07/13/mujoco-model-yourself.html} [Accessed: 28/Jul/2025]}
}

```