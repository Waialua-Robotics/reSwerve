# reSwerve

**Autonomous**

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
an excerpt from the 0-to-autonomous program. You should be able to replace the current get autonomous function with this and enable odometry if it is missing.

**Swerve Module Class**
- Initially, the absolute encoder was read once in to initialiese the integrated motor encoders. However, due to observed position jitter when perpetually reading
from the integrated encoder, now, only the absolute encoder provides wheel angle feedback: Annotations are present in the getAngle() method. The cause is unkonw but this 
could be a problem that is solved along with the main trouble surrounding pivot conversions and frames of reference.

**Conversions**

- Conversions are generally named in the convention *DEVICE_toUNIT*. This simply indicates that the conversion is for a device of the name *DEVICE*, and the procedure 
converts to the *UNIT*. I think we will be able to get away with this because conversions are typically used to translate a device's native units to some other unit with 
physical implications. Regardless, this code does **NOT** work.
- If I had to guess, the conversions are the root of the program's problems and the primary cause of my clinical depression. Im not even sure if you want to organize 
the conversions such that they exist as public static functions in their own file but do whatever works. 

**Drive**

- The last thing Mr.Wood and I worked on was the creating a very simple control scheme with what was avaliable. The code that currently reads the x, y, and rotation 
values is conditional and the outputs are as follows: move horizantal at a constant speed, move vertical at a constant speed, move a combination of the two at a constant speed, 
and don't move at all (which initiates the stop function, turning all the wheels to where they think zero is).
- I would conduct most of the debugging within this class. To clarify, this means making new public debugging variables in the module class and reading all of said 
individual variables through the drive class. This is because printing to the smartdashboard from the module class would indicate that all objects of the module class 
would be overwriting one number on the smartdashboard. I think the conversions are going to need the most observing but working from the beginning with the kinematics 
in the drive class would also be a good option.

**Procedure**

- I would begin by adding the probe variables that I mentioned where you deem necessary, and looking at the values that go into and out of each function and operator.
- You may want to especially observe the angles that are produced by the kinematics for a repective angle of the joystick, and try to fix the "kinematicsToAngle" function if the values don't make sense.
