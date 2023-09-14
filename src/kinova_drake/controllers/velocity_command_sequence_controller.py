from kinova_drake.kinova_station import EndEffectorTarget
from .velocity_controller import *
from .velocity_command_sequence import *

class VelocityCommandSequenceController(VelocityController):
    """
    A simple controller which tracks a sequence of target end-effector velocities
    and gripper states (open or closed). See VelocityController for IO details. 

    Sends exclusively gripper position commands and end-effector twist/wrench commands.
    """
    def __init__(self, command_sequence, 
                       command_type = EndEffectorTarget.kTwist, 
                       Kp = 10*np.eye(6),
                       Kd = 2*np.sqrt(10)*np.eye(6)):

        VelocityController.__init__(self, command_type=command_type)

        self.cs = command_sequence   # a CommandSequence object

        self.Kp = Kp  # PD gains
        self.Kd = Kd

    def CalcGripperCommand(self, context, output):
        t = context.get_time()

        if self.cs.gripper_closed(t):
            cmd_pos = np.array([1.0])  # closed 
        else:
            cmd_pos = np.array([0.0])  # open

        output.SetFromVector(cmd_pos)

    def CalcEndEffectorCommand(self, context, output):
        """
        Compute and send an end-effector twist command. This is just a
        simple PD controller.
        """
        t = context.get_time()

        # Get target end-effector velocity and twist
        target_velocity = self.cs.target_velocity(t)
        target_twist = np.zeros(6)

        # print("target_velocity: ")
        # print(target_velocity)

        # Get current end-effector pose and twist
        current_velocity = self.ee_velocity_port.Eval(context)
        current_twist = self.ee_twist_port.Eval(context)

        # Compute pose and twist errors
        twist_err = target_twist - current_twist
        velocity_err = target_velocity - current_velocity

        # print("velocity_err: ")
        # print(velocity_err)

        # Use rotation matrices to compute the difference between current and
        # desired end-effector orientations. This helps avoid gimbal lock as well 
        # as issues like taking the shortest path from theta=0 to theta=2*pi-0.1
        # R_current = RotationMatrix(RollPitchYaw(current_pose[:3]))
        # R_target = RotationMatrix(RollPitchYaw(target_pose[:3]))
        # R_err = R_target.multiply(R_current.transpose())
        # pose_err[:3] = RollPitchYaw(R_err).vector()

        # Set command (i.e. end-effector twist or wrench) using a PD controller
        cmd = self.Kp@velocity_err + self.Kd@twist_err

        output.SetFromVector(cmd)

    def ConnectToStation(self, builder, station):
        """
        Connect inputs and outputs of this controller to the given kinova station (either
        hardware or simulation). 
        """
        builder.Connect(                                  # Send commands to the station
                self.GetOutputPort("ee_command"),
                station.GetInputPort("ee_target"))
        builder.Connect(
                self.GetOutputPort("ee_command_type"),
                station.GetInputPort("ee_target_type"))
        builder.Connect(
                self.GetOutputPort("gripper_command"),
                station.GetInputPort("gripper_target"))
        builder.Connect(
                self.GetOutputPort("gripper_command_type"),
                station.GetInputPort("gripper_target_type"))

        builder.Connect(                                        # Send estimated state information
                station.GetOutputPort("estimated_ee_velocity"), # to the controller
                self.GetInputPort("ee_velocity"))
        builder.Connect(
                station.GetOutputPort("measured_ee_twist"),
                self.GetInputPort("ee_twist"))