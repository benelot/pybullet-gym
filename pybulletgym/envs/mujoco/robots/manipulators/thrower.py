from pybulletgym.envs.mujoco.robots.robot_bases import MJCFBasedRobot


class Thrower(MJCFBasedRobot):
    min_target_placement_radius = 0.1
    max_target_placement_radius = 0.8
    min_object_placement_radius = 0.1
    max_object_placement_radius = 0.8

    def __init__(self):
        MJCFBasedRobot.__init__(self, 'thrower.xml', 'body0', action_dim=7, obs_dim=48)

    def robot_specific_reset(self, bullet_client):
        # parts
        self.fingertip = self.parts["r_wrist_roll_link"]
        self.target = self.parts["goal"]
        self.object = self.parts["ball"]

        # joints
        self.shoulder_pan_joint = self.jdict["r_shoulder_pan_joint"]
        self.shoulder_lift_joint = self.jdict["r_shoulder_lift_joint"]
        self.upper_arm_roll_joint = self.jdict["r_upper_arm_roll_joint"]
        self.elbow_flex_joint = self.jdict["r_elbow_flex_joint"]
        self.forearm_roll_joint = self.jdict["r_forearm_roll_joint"]
        self.wrist_flex_joint = self.jdict["r_wrist_flex_joint"]
        self.wrist_roll_joint = self.jdict["r_wrist_roll_joint"]

        self._object_hit_ground = False
        self._object_hit_location = None

        # reset position and speed of manipulator
        # TODO: Will this work or do we have to constrain this resetting in some way?
        self.shoulder_pan_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
        self.shoulder_lift_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
        self.upper_arm_roll_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
        self.elbow_flex_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
        self.upper_arm_roll_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
        self.wrist_flex_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)
        self.wrist_roll_joint.reset_current_position(self.np_random.uniform(low=-3.14, high=3.14), 0)

        self.zero_offset = np.array([0.45, 0.55, 0])
        self.object_pos = np.concatenate([
            self.np_random.uniform(low=-1, high=1, size=1),
            self.np_random.uniform(low=-1, high=1, size=1),
            self.np_random.uniform(low=-1, high=1, size=1)
        ])

        # make length of vector between min and max_object_placement_radius
        self.object_pos = self.object_pos \
                          / np.linalg.norm(self.object_pos) \
                          * self.np_random.uniform(low=self.min_object_placement_radius,
                                                   high=self.max_object_placement_radius, size=1)

        # reset object position
        self.parts["ball"].reset_pose(self.object_pos - self.zero_offset, np.array([0, 0, 0, 1]))

        self.target_pos = np.concatenate([
            self.np_random.uniform(low=-1, high=1, size=1),
            self.np_random.uniform(low=-1, high=1, size=1),
            self.np_random.uniform(low=-1, high=1, size=1)
        ])

        # make length of vector between min and max_target_placement_radius
        self.target_pos = self.target_pos \
                          / np.linalg.norm(self.target_pos) \
                          * self.np_random.uniform(low=self.min_target_placement_radius,
                                                   high=self.max_target_placement_radius, size=1)

        self.parts["goal"].reset_pose(self.target_pos - self.zero_offset, np.array([0, 0, 0, 1]))

    def apply_action(self, a):
        assert (np.isfinite(a).all())
        self.shoulder_pan_joint.set_motor_torque(0.05 * float(np.clip(a[0], -1, +1)))
        self.shoulder_lift_joint.set_motor_torque(0.05 * float(np.clip(a[1], -1, +1)))
        self.upper_arm_roll_joint.set_motor_torque(0.05 * float(np.clip(a[2], -1, +1)))
        self.elbow_flex_joint.set_motor_torque(0.05 * float(np.clip(a[3], -1, +1)))
        self.upper_arm_roll_joint.set_motor_torque(0.05 * float(np.clip(a[4], -1, +1)))
        self.wrist_flex_joint.set_motor_torque(0.05 * float(np.clip(a[5], -1, +1)))
        self.wrist_roll_joint.set_motor_torque(0.05 * float(np.clip(a[6], -1, +1)))

    def calc_state(self):
        qpos = np.array([j.current_position() for j in self.ordered_joints]).flatten()  # shape (16,)
        qvel = np.array([j.current_relative_position() for j in self.ordered_joints]).flatten()  # shape (15,)
        wrist_roll_link_body_com = self.fingertip.pose().xyz()  # shape (3,)
        ball_body_com = self.object.pose().xyz()  # shape (3,)
        goal_body_com = self.target.pose().xyz()  # shape (3,)

        return np.concatenate([
            qpos.flat[:7],  # self.sim.data.qpos.flat[:7],
            qvel.flat[:7],  # self.sim.data.qvel.flat[:7],
            wrist_roll_link_body_com,  # self.get_body_com("r_wrist_roll_link"),
            ball_body_com,  # self.get_body_com("ball"),
            goal_body_com, # self.get_body_com("goal"),
        ])
