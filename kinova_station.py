from pydrake.all import *
from helpers import EndEffectorTargetType

class KinovaStation(Diagram):
    """
    A template system diagram for controlling 7 DoF Kinova Gen3 robot, modeled 
    after Drake's ManipulationStation, but with the kinova instead of a kuka arm.
   
                               ---------------------------------
                               |                               |
                               |                               |
                               |                               |
                               |                               | --> measured_arm_position
                               |                               | --> measured_arm_velocity
    ee_target ---------------> |         KinovaStation         | --> measured_arm_torque
    ee_target_type ----------> |                               |
                               |                               |
                               |                               | --> measured_ee_pose
                               |                               | --> measured_ee_twist
                               |                               | --> measured_ee_wrench
    gripper_target ----------> |                               |
    gripper_target_type -----> |                               |
                               |                               | --> measured_gripper_position
                               |                               | --> measured_gripper_velocity
                               |                               |
                               |                               | --> camera_rgb_image (TODO)
                               |                               | --> camera_depth_image (TODO)
                               |                               |
                               |                               |
                               ---------------------------------

    The input ee_target can be a desired end-effector pose, twist or wrench, as specified by
    ee_target_type. 

    Similarly, the gripper_target can be a desired gripper position or velocity, as specified
    by gripper_target_type. 
   
    """
    def __init__(self, time_step=0.002):
        Diagram.__init__(self)
        self.set_name("manipulation_station")

        self.builder = DiagramBuilder()

        self.scene_graph = self.builder.AddSystem(SceneGraph())
        self.scene_graph.set_name("scene_graph")

        self.plant = self.builder.AddSystem(MultibodyPlant(time_step=time_step))
        self.plant.RegisterAsSourceForSceneGraph(self.scene_graph)
        self.plant.set_name("plant")

        # A separate plant which only has access to the robot arm + gripper mass,
        # and not any other objects in the scene
        self.controller_plant = MultibodyPlant(time_step=time_step)

    def Finalize(self):
        """
        Do some final setup stuff. Must be called after making modifications
        to the station (e.g. adding arm, gripper, manipulands) and before using
        this diagram as a system. 
        """
        self.plant.Finalize()
        self.controller_plant.Finalize()
        
        # Set up the scene graph
        self.builder.Connect(
                self.scene_graph.get_query_output_port(),
                self.plant.get_geometry_query_input_port())
        self.builder.Connect(
                self.plant.get_geometry_poses_output_port(),
                self.scene_graph.get_source_pose_port(self.plant.get_source_id()))

        # Create controller
        cartesian_controller = self.builder.AddSystem(CartesianController(
                                        self.controller_plant,
                                        self.controller_arm))
        cartesian_controller.set_name("cartesian_controller")

        # End effector target and target type go to the controller
        self.builder.ExportInput(cartesian_controller.ee_target_port,
                                 "ee_target")
        self.builder.ExportInput(cartesian_controller.ee_target_type_port,
                                 "ee_target_type")

        # Output measured arm position and velocity
        demux = self.builder.AddSystem(Demultiplexer(
                                        self.plant.num_multibody_states(self.arm),
                                        self.plant.num_positions(self.arm)))
        demux.set_name("demux")
        self.builder.Connect(
                self.plant.get_state_output_port(self.arm),
                demux.get_input_port(0))
        self.builder.ExportOutput(
                demux.get_output_port(0),
                "measured_arm_position")
        self.builder.ExportOutput(
                demux.get_output_port(1),
                "measured_arm_velocity")
        
        # Measured arm position and velocity are sent to the controller
        self.builder.Connect(
                demux.get_output_port(0),
                cartesian_controller.arm_position_port)
        self.builder.Connect(
                demux.get_output_port(1),
                cartesian_controller.arm_velocity_port)

        # Torques from controller go to the simulated plant
        self.builder.Connect(
                cartesian_controller.GetOutputPort("applied_arm_torque"),
                self.plant.get_actuation_input_port(self.arm))

        # Controller outputs measured arm torques, end-effector pose, end-effector twist
        self.builder.ExportOutput(
                cartesian_controller.GetOutputPort("applied_arm_torque"),
                "measured_arm_torques")
        self.builder.ExportOutput(
                cartesian_controller.GetOutputPort("measured_ee_pose"),
                "measured_ee_pose")
        self.builder.ExportOutput(
                cartesian_controller.GetOutputPort("measured_ee_twist"),
                "measured_ee_twist")
        
        # Create gripper controller
        Kp_gripper = 10*np.ones((2,1))
        Kd_gripper = 2*np.sqrt(Kp_gripper)
        Ki_gripper = 0*Kp_gripper
        gripper_controller = self.builder.AddSystem(
                                    PidController(Kp_gripper, Ki_gripper, Kd_gripper))
        gripper_controller.set_name("gripper_controller")

        # Connect gripper controller to the diagram
        mux = self.builder.AddSystem(Multiplexer([2,2]))
        mux.set_name("mux")

        self.builder.ExportInput(
                mux.get_input_port(0),
                "target_gripper_position")
        self.builder.ExportInput(
                mux.get_input_port(1),
                "target_gripper_velocity")

        self.builder.Connect(       # actual gripper state
                self.plant.get_state_output_port(self.gripper),
                gripper_controller.get_input_port(0))
        self.builder.Connect(       # desired gripper state
                mux.get_output_port(),
                gripper_controller.get_input_port(1))

        self.builder.Connect(
                gripper_controller.get_output_port(),
                self.plant.get_actuation_input_port(self.gripper))

        # Send gripper position and velocity as an output
        demux2 = self.builder.AddSystem(Demultiplexer(
                                        self.plant.num_multibody_states(self.gripper),
                                        self.plant.num_positions(self.gripper)))
        demux2.set_name("demux2")
        
        self.builder.Connect(
                self.plant.get_state_output_port(self.gripper),
                demux2.get_input_port())
        self.builder.ExportOutput(
                demux2.get_output_port(0),
                "measured_gripper_position")
        self.builder.ExportOutput(
                demux2.get_output_port(1),
                "measured_gripper_velocity")

        # Compute and output end-effector wrenches based on measured joint torques
        wrench_calculator = self.builder.AddSystem(EndEffectorWrenchCalculator(
                self.controller_plant,
                self.controller_plant.GetFrameByName("end_effector_link")))
        wrench_calculator.set_name("wrench_calculator")

        self.builder.Connect(
                demux.get_output_port(0),
                wrench_calculator.GetInputPort("joint_positions"))
        self.builder.Connect(
                demux.get_output_port(1),
                wrench_calculator.GetInputPort("joint_velocities"))
        self.builder.Connect(
                cartesian_controller.GetOutputPort("applied_arm_torque"),
                wrench_calculator.GetInputPort("joint_torques"))


        self.builder.ExportOutput(
                wrench_calculator.get_output_port(),
                "measured_ee_wrench")

        # Build the diagram
        self.builder.BuildInto(self)

    def AddGround(self):
        """
        Add a flat ground with friction
        """
        X_BG = RigidTransform()
        surface_friction = CoulombFriction(
                static_friction = 0.7,
                dynamic_friction = 0.5)
        self.plant.RegisterCollisionGeometry(
                self.plant.world_body(),
                X_BG,
                HalfSpace(),
                "ground_collision",
                surface_friction)

    def AddArm(self):
        """
        Add the 7-dof gen3 arm to the system.
        """
        arm_urdf = "./models/gen3_7dof/urdf/GEN3_URDF_V12.urdf"
        self.arm = Parser(plant=self.plant).AddModelFromFile(arm_urdf, "arm")
        self.controller_arm = Parser(plant=self.controller_plant).AddModelFromFile(arm_urdf, "arm")

        # Fix the base of the arm to the world
        self.plant.WeldFrames(self.plant.world_frame(),
                              self.plant.GetFrameByName("base_link",self.arm))

        self.controller_plant.WeldFrames(self.controller_plant.world_frame(),
                                         self.controller_plant.GetFrameByName("base_link", self.controller_arm))

    def AddHandeGripper(self):
        """
        Add the Hand-e gripper to the system. The arm must be added first. 
        """
        # Add a gripper with actuation to the full simulated plant
        gripper_urdf = "./models/hande_gripper/urdf/robotiq_hande.urdf"
        self.gripper = Parser(plant=self.plant).AddModelFromFile(gripper_urdf,"gripper")

        self.plant.WeldFrames(self.plant.GetFrameByName("end_effector_link",self.arm),
                              self.plant.GetFrameByName("hande_base_link", self.gripper))

        # Add a gripper without actuation to the controller plant
        gripper_static_urdf = "./models/hande_gripper/urdf/robotiq_hande_static.urdf"
        static_gripper = Parser(plant=self.controller_plant).AddModelFromFile(
                                                                gripper_static_urdf,
                                                                "gripper")

        self.controller_plant.WeldFrames(
                self.controller_plant.GetFrameByName("end_effector_link",self.controller_arm),
                self.controller_plant.GetFrameByName("hande_base_link", static_gripper))
        
    def AddArmWithHandeGripper(self):
        """
        Add the 7-dof arm and a model of the hande gripper to the system.
        """
        self.AddArm()
        self.AddHandeGripper()

    def AddManipulandFromFile(self, model_file, X_WObject):
        pass

    def ConnectToDrakeVisualizer(self):
        visualizer_params = DrakeVisualizerParams(role=Role.kIllustration)
        DrakeVisualizer().AddToBuilder(builder=self.builder,
                                       scene_graph=self.scene_graph,
                                       params=visualizer_params)

class CartesianController(LeafSystem):
    """
    A controller which imitates the cartesian control mode of the Kinova gen3 arm.
 
                         -------------------------
                         |                       |
                         |                       |
    ee_target ---------> |  CartesianController  | ----> applied_arm_torque
    ee_target_type ----> |                       |
                         |                       |
                         |                       | ----> measured_ee_pose
    arm_position ------> |                       | ----> measured_ee_twist
    arm_velocity ------> |                       |
                         |                       |
                         |                       |
                         -------------------------

    The type of target is determined by ee_target_type, and can be
        EndEffectorTargetType.kPose,
        EndEffectorTargetType.kTwist,
        EndEffectorTargetType.kWrench.

    """
    def __init__(self, plant, arm_model):
        LeafSystem.__init__(self)

        self.plant = plant
        self.arm = arm_model
        self.context = self.plant.CreateDefaultContext()

        # Define input ports
        self.ee_target_port = self.DeclareVectorInputPort(
                                        "ee_target",
                                        BasicVector(6))
        self.ee_target_type_port = self.DeclareAbstractInputPort(
                                            "ee_target_type",
                                            AbstractValue.Make(EndEffectorTargetType.kPose))

        self.arm_position_port = self.DeclareVectorInputPort(
                                        "arm_position",
                                        BasicVector(self.plant.num_positions(self.arm)))
        self.arm_velocity_port = self.DeclareVectorInputPort(
                                        "arm_velocity",
                                        BasicVector(self.plant.num_velocities(self.arm)))

        # Define output ports
        self.DeclareVectorOutputPort(
                "applied_arm_torque",
                BasicVector(self.plant.num_actuators()),
                self.CalcArmTorques)

        self.DeclareVectorOutputPort(
                "measured_ee_pose",
                BasicVector(6),
                self.CalcEndEffectorPose)
        self.DeclareVectorOutputPort(
                "measured_ee_twist",
                BasicVector(6),
                self.CalcEndEffectorTwist)

        # Define some relevant frames
        self.world_frame = self.plant.world_frame()
        self.ee_frame = self.plant.GetFrameByName("end_effector_link")

        # Set joint limits (set self.{q,qd}_{min,max})
        self.GetJointLimits()
           
        # Store desired end-effector pose and corresponding joint
        # angles so we only run full IK when we need to
        self.last_ee_pose_target = None
        self.last_q_target = None
    
    def GetJointLimits(self):
        """
        Iterate through self.plant to establish joint angle
        and velocity limits. 

        Sets:

            self.q_min
            self.q_max
            self.qd_min
            self.qd_max

        """
        q_min = []
        q_max = []
        qd_min = []
        qd_max = []

        joint_indices = self.plant.GetJointIndices(self.arm)

        for idx in joint_indices:
            joint = self.plant.get_joint(idx)
            
            if joint.type_name() == "revolute":  # ignore the joint welded to the world
                q_min.append(joint.position_lower_limit())
                q_max.append(joint.position_upper_limit())
                qd_min.append(joint.velocity_lower_limit())  # note that higher limits
                qd_max.append(joint.velocity_upper_limit())  # are availible in cartesian mode

        self.q_min = np.array(q_min)
        self.q_max = np.array(q_max)
        self.qd_min = np.array(qd_min)
        self.qd_max = np.array(qd_max)

    def CalcEndEffectorPose(self, context, output):
        """
        This method is called each timestep to determine the end-effector pose
        """
        q = self.arm_position_port.Eval(context)
        qd = self.arm_velocity_port.Eval(context)
        self.plant.SetPositions(self.context,q)
        self.plant.SetVelocities(self.context,qd)

        # Compute the rigid transform between the world and end-effector frames
        X_ee = self.plant.CalcRelativeTransform(self.context,
                                                self.world_frame,
                                                self.ee_frame)

        ee_pose = np.hstack([RollPitchYaw(X_ee.rotation()).vector(), X_ee.translation()])
        output.SetFromVector(ee_pose)
    
    def CalcEndEffectorTwist(self, context, output):
        """
        This method is called each timestep to determine the end-effector twist
        """
        q = self.arm_position_port.Eval(context)
        qd = self.arm_velocity_port.Eval(context)
        self.plant.SetPositions(self.context,q)
        self.plant.SetVelocities(self.context,qd)

        # Compute end-effector Jacobian
        J = self.plant.CalcJacobianSpatialVelocity(self.context,
                                                   JacobianWrtVariable.kV,
                                                   self.ee_frame,
                                                   np.zeros(3),
                                                   self.world_frame,
                                                   self.world_frame)

        ee_twist = J@qd
        output.SetFromVector(ee_twist)
    
    def CalcArmTorques(self, context, output):
        """
        This method is called each timestep to determine output torques
        """
        q = self.arm_position_port.Eval(context)
        qd = self.arm_velocity_port.Eval(context)
        self.plant.SetPositions(self.context,q)
        self.plant.SetVelocities(self.context,qd)

        # Some dynamics computations
        tau_g = -self.plant.CalcGravityGeneralizedForces(self.context)

        # Indicate what type of command we're recieving
        target_type = self.ee_target_type_port.Eval(context)

        if target_type == EndEffectorTargetType.kWrench:
            # Compute joint torques consistent with the desired wrench
            wrench_des = self.ee_target_port.Eval(context)
            print("got a wrench")
            pass
        elif target_type == EndEffectorTargetType.kTwist:
            # Compue joint torques consistent with the desired twist
            twist_des = self.ee_target_port.Eval(context)
            print("got a twist")
            
            ## Use DoDifferentialInverseKinematics to determine desired q, qd
            #X_WE_desired = RigidTransform(RollPitchYaw(rpy_xyz_des[:3]),
            #                              rpy_xyz_des[-3:]).GetAsIsometry3()

            #params = DifferentialInverseKinematicsParameters(self.plant.num_positions(),
            #                                                 self.plant.num_velocities())
            #params.set_timestep(0.005)
            #params.set_joint_velocity_limits((self.qd_min, self.qd_max))
            #params.set_joint_position_limits((self.q_min, self.q_max))

            #result = DoDifferentialInverseKinematics(self.plant,
            #                                         self.context,
            #                                         X_WE_desired,
            #                                         self.ee_frame,
            #                                         params)
            pass
        elif target_type == EndEffectorTargetType.kPose:
            # Compute joint torques which move the end effector to the desired pose
            rpy_xyz_des = self.ee_target_port.Eval(context)

            # Only do a full inverse kinematics solve if the target pose has changed
            if (rpy_xyz_des != self.last_ee_pose_target).any():
                
                X_WE_des = RigidTransform(RollPitchYaw(rpy_xyz_des[:3]),
                                          rpy_xyz_des[-3:])         
          
                # First compute joint angles consistent with the desired pose using Drake's IK.
                # This sets up a nonconvex optimization problem to find joint angles consistent
                # with the given constraints
                ik = InverseKinematics(self.plant,self.context)
                ik.AddPositionConstraint(self.ee_frame,
                                         [0,0,0],
                                         self.world_frame,
                                         X_WE_des.translation(), 
                                         X_WE_des.translation())
                ik.AddOrientationConstraint(self.ee_frame,
                                            RotationMatrix(),
                                            self.world_frame,
                                            X_WE_des.rotation(),
                                            0.001)

                prog = ik.get_mutable_prog()
                q_var = ik.q()
                prog.AddQuadraticErrorCost(np.eye(len(q_var)), q, q_var)
                prog.SetInitialGuess(q_var, q)
                result = Solve(ik.prog())

                if not result.is_success():
                    print("Inverse Kinematics Failed!")
                    q_nom = np.zeros(7)
                else:
                    q_nom = result.GetSolution(q_var)

                    # Save the results of this solve for later
                    self.last_ee_pose_target = rpy_xyz_des
                    self.last_q_target = q_nom

            else:
                q_nom = self.last_q_target

            qd_nom = np.zeros(7)

            # Use PD controller to map desired q, qd to desired qdd
            Kp = 1*np.eye(7)
            Kd = 2*np.sqrt(Kp)  # critical damping
            qdd_nom = Kp@(q_nom - q) + Kd@(qd_nom - qd)

            # Compute joint torques consistent with these desired qdd
            f_ext = MultibodyForces(self.plant)
            tau = tau_g + self.plant.CalcInverseDynamics(self.context, qdd_nom, f_ext)

        else:
            raise RuntimeError("Invalid target type %s" % target_type)

        output.SetFromVector(tau)

class EndEffectorWrenchCalculator(LeafSystem):
    """
    A simple system which takes as input joint torques and outputs the corresponding
    wrench applied to the end-effector. 

                       ---------------------------------
                       |                               |
                       |                               |
                       |                               |
    joint_positions -> |  EndEffectorWrenchCalculator  | ---> end_effector_wrench
    joint_angles ----> |                               | 
    joint_torques ---> |                               |
                       |                               |
                       |                               |
                       ---------------------------------
    """
    def __init__(self, plant, end_effector_frame):
        LeafSystem.__init__(self)

        self.plant = plant
        self.context = self.plant.CreateDefaultContext()
        self.ee_frame = end_effector_frame

        # Inputs are joint positions, angles and torques
        self.q_port = self.DeclareVectorInputPort(
                                "joint_positions",
                                BasicVector(plant.num_positions()))
        self.v_port = self.DeclareVectorInputPort(
                                "joint_velocities",
                                BasicVector(plant.num_velocities()))
        self.tau_port = self.DeclareVectorInputPort(
                                "joint_torques",
                                BasicVector(plant.num_actuators()))

        # Output is applied wrench at the end-effector
        self.DeclareVectorOutputPort(
                "end_effector_wrench",
                BasicVector(6),
                self.CalcEndEffectorWrench)

    def CalcEndEffectorWrench(self, context, output):
        # Gather inputs
        q = self.q_port.Eval(context)
        v = self.v_port.Eval(context)
        tau = self.tau_port.Eval(context)

        # Set internal model state
        self.plant.SetPositions(self.context, q)
        self.plant.SetVelocities(self.context, v)

        # Some dynamics computations
        M = self.plant.CalcMassMatrixViaInverseDynamics(self.context)
        tau_g = -self.plant.CalcGravityGeneralizedForces(self.context)

        # Compute end-effector jacobian
        J = self.plant.CalcJacobianSpatialVelocity(self.context,
                                                   JacobianWrtVariable.kV,
                                                   self.ee_frame,
                                                   np.zeros(3),
                                                   self.plant.world_frame(),
                                                   self.plant.world_frame())

        # Compute jacobian pseudoinverse
        Minv = np.linalg.inv(M)
        Lambda = np.linalg.inv(J@Minv@J.T)
        Jbar = Lambda@J@Minv

        # Compute wrench (spatial force) applied at end-effector
        w = Jbar@(tau-tau_g)

        output.SetFromVector(w)
