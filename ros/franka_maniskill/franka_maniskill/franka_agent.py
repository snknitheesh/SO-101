from mani_skill.agents.base_agent import BaseAgent
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent
import os
import sapien
from mani_skill.utils import sapien_utils, common
from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.utils.registration import register_env

@register_agent()
class so101agent(BaseAgent):
    uid = "franka"
    home_path = os.environ["HOME"]
    urdf_path = "/home/zozo/zobot_ws/le-robot/so101-moveit-isaacsim_ws/src/franka_maniskill/urdfs/fr3_franka_hand.urdf"
    
    urdf_config = dict(
        urdf_path=urdf_path,
        base_link="base",  
        end_effector_link="jaw", 
        joint_names_arm=[
            "fr3_joint1",
            "fr3_joint2",
            "fr3_joint3",
            "fr3_joint4",
            "fr3_joint5",
            "fr3_joint6",
            "fr3_joint7",
        ],
        joint_names_gripper=[
            "fr3_finger_joint1",
        ],
        joint_limits={
            "fr3_joint1":        [-2.8973, 2.8973],
            "fr3_joint2":        [-1.7628, 1.7628],
            "fr3_joint3":        [-2.8973, 2.8973],
            "fr3_joint4":        [-3.0718, -0.0698],
            "fr3_joint5":        [-2.8973, 2.8973],
            "fr3_joint6":        [-0.0175, 3.7525],
            "fr3_joint7":        [-2.8973, 2.8973],
            "fr3_finger_joint1": [0.0, 0.04],
        }
    )
    stiffness = {
        "fr3_joint1":        100.0,
        "fr3_joint2":        100.0,
        "fr3_joint3":        100.0,
        "fr3_joint4":        100.0,
        "fr3_joint5":        100.0,
        "fr3_joint6":        100.0,
        "fr3_joint7":        100.0,
        "fr3_finger_joint1": 100.0,
    }
    damping = {
        "fr3_joint1":        0.5,
        "fr3_joint2":        0.5,
        "fr3_joint3":        0.5,
        "fr3_joint4":        0.5,
        "fr3_joint5":        0.5,
        "fr3_joint6":        0.5,
        "fr3_joint7":        0.5,
        "fr3_finger_joint1": 0.5,
    }
    force_limits = {
        "fr3_joint1":        10.0,
        "fr3_joint2":        10.0,
        "fr3_joint3":        10.0,
        "fr3_joint4":        10.0,
        "fr3_joint5":        10.0,
        "fr3_joint6":        10.0,
        "fr3_joint7":        10.0,
        "fr3_finger_joint1": 10.0,
    }
    default_joint_positions = {
        "fr3_joint1":        0.001,
        "fr3_joint2":       -0.025,
        "fr3_joint3":        0.001,
        "fr3_joint4":       -0.152,
        "fr3_joint5":        0.001,
        "fr3_joint6":        0.544,
        "fr3_joint7":       -0.001,
        "fr3_finger_joint1": 0.000,
    }
    