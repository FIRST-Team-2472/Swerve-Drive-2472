package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
        
        //(Max)create objects of subsytems
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

        //TODO (Max) Why the hell are they called OIConstants?
    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        //sets up the defalt command for the swerve subsystem. Defalut commands run if no other commands are set
        //the () -> are lambda expressions.
        //(Max) so the lambda is sending over a method with preset conditions
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        //TODO (Max) Wait so the buttons will be set up always even in autonmous?
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        //reseting button for IMU. Usefull for change field orentation forward direction
        //(Max) What the hell is creating the ghost buttonNumber?
        //(Max) Guess buttons are now class
        //(Max) What is the point of Lambda if it's paramenter is blank does it turn it into a command?
        new JoystickButton(driverJoytick, 5).whenPressed(() -> swerveSubsystem.zeroHeading());
        new JoystickButton(driverJoytick, 3).whenPressed(() -> swerveSubsystem.resetOdometry(new Pose2d()));
    }

    //generates a path via points
    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        //TODO (Max) GET RID OF THE K IN FRONT OF EVERY CONSTANT
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        //(Max) why do ghost parameter names only show sometimes?
        //(Max) Genrates trajectory need to feed start point, a sereris of inbtween points, and end point
        //(Max) Midpoints it does what angle it wants for you
        //TODO (Max) shouldn't the start point angle be the current angle of robot
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 0), new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        //(Max) what is a PIDController and thetaController is a bad var name
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                //(Max) why is swerveSubsystem::getPose not swerveSubsystem.getPose()?
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        //(Max) creates a Command list that will reset the Odometry to the set start point, then move the path, then stop
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}
