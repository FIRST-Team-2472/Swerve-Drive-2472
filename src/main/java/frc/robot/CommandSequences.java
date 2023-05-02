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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class CommandSequences {
    Pose2d[] exampleNodes = new Pose2d[1];

    public CommandSequences() {
        exampleNodes[0] = simplePose(1, 0, 180);
    }

  
    public Command driveToPointAndRotate(SwerveSubsystem swerveSubsystem, Pose2d pose) {
        Pose2d drivePos = new Pose2d(swerveSubsystem.positivePosToDrivePos(pose.getTranslation()), pose.getRotation());
        return new SwerveDriveToPointCmd(swerveSubsystem, drivePos);
    }

    public Command defualtAuto(SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.resetOdometryFromPositivePos(new Pose2d());

        return new SequentialCommandGroup(
            genratePath(swerveSubsystem, new Pose2d(), List.of(),  exampleNodes[0])
        );
    }

    // generates a path via points
    private Command genratePath(SwerveSubsystem swerveSubsystem, Pose2d startPoint, List<Translation2d> midPoints,
            Pose2d endPoint) {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

        startPoint = new Pose2d(swerveSubsystem.positivePosToDrivePos(startPoint.getTranslation()), startPoint.getRotation());
        endPoint = new Pose2d(swerveSubsystem.positivePosToDrivePos(endPoint.getTranslation()), endPoint.getRotation());
        midPoints.forEach(i -> i = swerveSubsystem.positivePosToDrivePos(i));

        // 2. Generate trajectory
        // Genrates trajectory need to feed start point, a series of inbetween points,
        // and end point
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                startPoint,
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
                // swerveSubsystm::getPose is same as () -> swerveSubsystem.getPose()
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        // creates a Command list that will reset the Odometry, then move the path, then
        // stop
        return new SequentialCommandGroup(
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }

    public Pose2d simplePose(double x, double y, double angleDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }

}
