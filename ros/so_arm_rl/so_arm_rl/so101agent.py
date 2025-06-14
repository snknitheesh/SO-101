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

    uid = "so101agent"
    home_path = os.environ["HOME"]
    urdf_path = "/home/zozo/zobot_ws/le-robot/so101-moveit-isaacsim_ws/src/so_arm_description/urdf/so101_new_calib.urdf"
    
    urdf_config = dict(
        urdf_path=urdf_path,
        base_link="base",  
        end_effector_link="jaw", 
        joint_names_arm=[
            "Rotation",      # base to shoulder
            "Pitch",         # shoulder to upper_arm
            "Elbow",         # upper_arm to lower_arm
            "Wrist_Pitch",   # lower_arm to wrist
            "Wrist_Roll",    # wrist to gripper
        ],
        joint_names_gripper=[    
            "Jaw"            # gripper to jaw
        ],
        joint_limits={
            "Rotation":    [-1.91986, 1.91986],
            "Pitch":       [-1.74533, 1.74533],
            "Elbow":       [-1.74533, 1.5708],
            "Wrist_Pitch": [-1.65806, 1.65806],
            "Wrist_Roll":  [-2.79253, 2.79253],
            "Jaw":         [-0.174533, 1.74533]
        }
    )
    stiffness = {
        "Rotation":    100.0,
        "Pitch":       100.0,
        "Elbow":       100.0,
        "Wrist_Pitch": 100.0,
        "Wrist_Roll":  100.0,
        "Jaw":         100.0
    }
    damping = {
        "Rotation":    0.5,
        "Pitch":       0.5,
        "Elbow":       0.5,
        "Wrist_Pitch": 0.5,
        "Wrist_Roll":  0.5,
        "Jaw":         0.5
    }
    force_limits = {
        "Rotation":    10.0,
        "Pitch":       10.0,
        "Elbow":       10.0,
        "Wrist_Pitch": 10.0,
        "Wrist_Roll":  10.0,
        "Jaw":         10.0
    }
    default_joint_positions = {
        "Rotation":    0.0,
        "Pitch":       0.0,
        "Elbow":       0.0,
        "Wrist_Pitch": 0.0,
        "Wrist_Roll":  0.0,
        "Jaw":         0.0
    }
    
    
    @property
    def tcp(self):
        # Find the link named "jaw" in the robot's links
        for link in self.robot.get_links():
            if link.name == "jaw":
                return link
        raise RuntimeError("TCP link 'jaw' not found in robot links")