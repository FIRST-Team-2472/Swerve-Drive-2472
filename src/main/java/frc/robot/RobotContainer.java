package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveDriveToPointCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
        
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final CommandSequences commandSequences = new CommandSequences();

    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final Joystick secodaryDriverJoystick = new Joystick(OIConstants.kSecondaryDriverControllerPort);

    public RobotContainer() {
        //sets up the defalt command for the swerve subsystem. Defalut commands run if no other commands are set
        //the () -> are lambda expressions.
        //lambda is sending over a method
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> secodaryDriverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !secodaryDriverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();
    }

    private void configureButtonBindings() {        
        //reseting button for IMU. Usefull for change field orentation forward direction
        new JoystickButton(secodaryDriverJoystick, 3).whileTrue(new SwerveDriveToPointCmd(swerveSubsystem, new Pose2d(1, 1, Rotation2d.fromDegrees(0))));
        new JoystickButton(driverJoytick, 2).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        new JoystickButton(driverJoytick, 3).onTrue(new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d())));
    }

    //generates a path via points
    public Command getAutonomousCommand() {
        return commandSequences.autoBlue5(swerveSubsystem);
    }
}
