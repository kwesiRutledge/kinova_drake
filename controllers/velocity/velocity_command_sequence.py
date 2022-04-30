##
#
# Defines an object for sending sequences of target 
# end-effector velocities and gripper states (open or closed), indexed 
# by time. 
#
##

import numpy as np

class VelocityCommand():
    """
    A simple object describing a single target end-effector velocity, 
    gripper state, and duration. 
    """
    def __init__(self, target_velocity, gripper_closed, duration, name=None ):
        """
        Parameters:

            target_velocity : a 6D vector (np array) describing the desired end-effector velocity
            gripper_closed  : boolean, true if the gripper is to be closed
            duration        : the number of seconds allocated to achieving this command. 
            name (optional) : a string describing this command

        """
        self.target_velocity = target_velocity
        self.gripper_closed = gripper_closed
        self.duration = duration
       
        if name is not None:
            self.name = name
        else:
            self.name = "command"

    def __str__(self):
        string = "%s: \n" % self.name
        string += "    target velocity: %s\n" % self.target_velocity
        string += "    gripper_closed : %s\n" % self.gripper_closed
        string += "    duration       : %s\n" % self.duration

        return string

class VelocityCommandSequence():
    """
    An object which describes a sequence of VelocityCommands. Basically a fancy list 
    with user-friendly ways of accessing the command data at any given time. 
    """
    def __init__(self, command_list):
        self.commands = []       # stores the commands
        self.start_times = [0]   # stores the time each command is to start at

        for command in command_list:
            self.append(command)

    def __str__(self):
        string = ""
        for command in self.commands:
            string += command.__str__()
        return string

    def append(self, command):
        """ Add a command to the sequence. """
        self.commands.append(command)
        self.start_times.append(self.start_times[-1] + command.duration)

    def current_command(self, t):
        """
        Return the command that is active at the given time, t. 
        """
        assert len(self.commands) > 0 , "Empty command sequence. "
        assert t >= 0, "Only positive times allowed."

        print(t)

        # Look through the spaces in start times to try to place t
        for i in range(len(self.commands)):
            if (self.start_times[i] <= t) and (t < self.start_times[i+1]):
                return self.commands[i]

        # If t is not in any of those intervals, the last command holds
        return self.commands[-1]

    def target_velocity(self, t):
        return self.current_command(t).target_velocity

    def gripper_closed(self, t):
        return self.current_command(t).gripper_closed

    def total_duration(self):
        return self.start_times[-1]