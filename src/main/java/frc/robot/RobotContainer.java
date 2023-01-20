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
import edu.wpi.first.wpilibj.XboxController;
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
        
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final CommandSequences commandSequences = new CommandSequences();

    private final XboxController driverJoytick = new XboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        //sets up the defalt command for the swerve subsystem. Defalut commands run if no other commands are set
        //the () -> are lambda expressions.
        //lambda is sending over a method
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        //this buttons sould only only muniplate parts that don't move robot
        
        //reseting button for IMU. Usefull for change field orentation forward direction
        new JoystickButton(driverJoytick, 1).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        new JoystickButton(driverJoytick, 2).onTrue(new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d())));
    }

    //generates a path via points
    public Command getAutonomousCommand() {
        return commandSequences.autonomousExample(swerveSubsystem);
    }
}
