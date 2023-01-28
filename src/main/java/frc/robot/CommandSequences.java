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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



public class CommandSequences {
    
    //generates a path via points
    private Command genratePath(SwerveSubsystem swerveSubsystem, List<Translation2d> midPoints, Pose2d endPoint) {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        //Genrates trajectory need to feed start point, a sereris of inbtween points, and end point
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                swerveSubsystem.getPose(),
                midPoints,
                endPoint,
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
                //swerveSubsystm::getPose is same as () -> swerveSubsystem.getPose()
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        //creates a Command list that will reset the Odometry, then move the path, then stop
        return new SequentialCommandGroup(
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }



    public Command autonomousExample( SwerveSubsystem swerveSubsystem) {
        return new SequentialCommandGroup(genratePath(swerveSubsystem, List.of(), new Pose2d(Units.inchesToMeters(-200),Units.inchesToMeters(16),Rotation2d.fromDegrees(180))),
        genratePath(swerveSubsystem, List.of(), new Pose2d(Units.inchesToMeters(-20),Units.inchesToMeters(0), new Rotation2d(0))),
        genratePath(swerveSubsystem, List.of(), new Pose2d(Units.inchesToMeters(76.19),Units.inchesToMeters(38.2), new Rotation2d(0))),
        genratePath(swerveSubsystem, List.of(), new Pose2d(Units.inchesToMeters(36.06),Units.inchesToMeters(0), new Rotation2d(0))));
    }

    public Command testAuto1( SwerveSubsystem swerveSubsystem) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(new Translation2d(1.87, .53), Rotation2d.fromDegrees(0)))),
                new InstantCommand(() -> swerveSubsystem.zeroHeading()),
                genratePath(swerveSubsystem, List.of(new Translation2d(2.85,0.92)), new Pose2d(7.09,0.92, Rotation2d.fromDegrees(180))));
                //genratePath(swerveSubsystem, List.of(new Translation2d(5.57,2.24)), new Pose2d(3.91,2.24, new Rotation2d(0))));
    }

    public Command testAuto2( SwerveSubsystem swerveSubsystem) {
        return new SequentialCommandGroup(
        genratePath(swerveSubsystem, List.of(), new Pose2d(2.87,0.53,Rotation2d.fromDegrees(0))),
        genratePath(swerveSubsystem, List.of(), new Pose2d(2.85,0.92, new Rotation2d(180))),
        genratePath(swerveSubsystem, List.of(), new Pose2d(7.09,0.92, new Rotation2d(180))),
        genratePath(swerveSubsystem, List.of(), new Pose2d(5.57,2.24, new Rotation2d(180))),
        genratePath(swerveSubsystem, List.of(), new Pose2d(3.91,2.24, new Rotation2d(180))),
        genratePath(swerveSubsystem, List.of(), new Pose2d(3.91,2.24, new Rotation2d(0))));
    }

 
}
