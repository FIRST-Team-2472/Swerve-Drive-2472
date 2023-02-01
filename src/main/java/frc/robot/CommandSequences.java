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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



public class CommandSequences {
    
    //generates a path via points
    private Command genratePath(SwerveSubsystem swerveSubsystem, Pose2d startPoint, List<Translation2d> midPoints, Pose2d endPoint) {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        //Genrates trajectory need to feed start point, a sereris of inbtween points, and end point
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

    public Command auto1( SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.resetOdometry(simplePose(1.87, .53, 0));
        swerveSubsystem.zeroHeading();

        return new SequentialCommandGroup(
                genratePath(swerveSubsystem, simplePose(1.87, .53, 0), List.of(simple2D(2.85, 0.92)), simplePose(7.09, 0.92, 180)),
                genratePath(swerveSubsystem, simplePose(5.09, 0.92, 180), List.of(simple2D(5.57, 2.24)), simplePose(3.91,2.24, 0))
        );
    }

    public Command auto2(SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.resetOdometry(simplePose(1.87, .53, 0));
        swerveSubsystem.zeroHeading();

        return new SequentialCommandGroup(
                genratePath(swerveSubsystem, simplePose(1.87, .53, 0), List.of(), simplePose(7.09, .92, 180)),
                genratePath(swerveSubsystem, simplePose(7.09, .92, 180), List.of(simple2D(2.46, .89)), simplePose(1.86, 1.62, 0))
        );
    }

    public Command auto3(SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.resetOdometry(simplePose(1.88, 4.97, 0));
        swerveSubsystem.zeroHeading();

        return new SequentialCommandGroup(
                genratePath(swerveSubsystem, simplePose(1.88, 4.97, 0), List.of(), simplePose(7.09, 4.60, 180)),
                genratePath(swerveSubsystem, simplePose(7.09, 4.60, 180), List.of(simple2D(5.45, 3.27)), simplePose(3.90, 3.27, 0))
        );
    }

    public Command auto4(SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.resetOdometry(simplePose(1.86, 4.97, 0));
        swerveSubsystem.zeroHeading();

        return new SequentialCommandGroup(
                genratePath(swerveSubsystem, simplePose(1.86, 4.97, 0), List.of(), simplePose(7.09, 4.60, 180)),
                genratePath(swerveSubsystem, simplePose(7.09, 4.60, 180), List.of(simple2D(2.43, 4.47)), simplePose(1.86, 3.86, 0))
        );
    }

    public Command auto5(SwerveSubsystem swerveSubsystem) {
        swerveSubsystem.resetOdometry(simplePose(1.86, 2.19, 0));
        swerveSubsystem.zeroHeading();

        return new SequentialCommandGroup(
                genratePath(swerveSubsystem, simplePose(1.86, 2.19, 0), List.of(simple2D(2.46, 2.14), simple2D(5.49, 2.14)), simplePose(7.09, 2.14, 180)),
                genratePath(swerveSubsystem, simplePose(7.09, 2.14, 180), List.of(simple2D(5.49, 2.14)), simplePose(3.91, 2.14, 180))
        );
    }

    public Pose2d simplePose(double x, double y, double angleDegrees) {
        return new Pose2d(x, -y, Rotation2d.fromDegrees(angleDegrees));
    }

    public Translation2d simple2D(double x, double y) {
        return new Translation2d(x, -y);
    }
}
