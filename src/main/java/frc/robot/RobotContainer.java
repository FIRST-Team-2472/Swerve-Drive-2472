// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.swerveExtras.DrivePose2d;
import frc.robot.subsystems.swerveExtras.PosPose2d;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  private final String defaultAuto = "Default Auto";

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Subsystems
  private final CommandSequences commandSequences = new CommandSequences();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick leftJoystick = new Joystick(OIConstants.kLeftJoystickPort);
  private final Joystick rightJoystick = new Joystick(OIConstants.kRightJoystickPort);
  private final XboxController xboxController = new XboxController(OIConstants.kXboxControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    m_chooser.addOption(defaultAuto, defaultAuto);

    ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");

    driverBoard.add("Auto choices", m_chooser).withWidget(BuiltInWidgets.kComboBoxChooser);

    // sets up the defalt command for the swerve subsystem. Defalut commands run if
    // no other commands are set
    // the () -> are lambda expressions. (so are exampleClass::exampleMethod)
    // lambda is sending over a method
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
        () -> -leftJoystick.getRawAxis(OIConstants.kLeftDriverYAxis),
        () -> leftJoystick.getRawAxis(OIConstants.kLeftDriverXAxis),
        () -> rightJoystick.getRawAxis(OIConstants.kRightDriverRotAxis),
        () -> leftJoystick.getRawButton(1)));
  }

  private void configureButtonBindings() {
    new JoystickButton(rightJoystick, 2).onTrue(new SwerveDriveToPointCmd(swerveSubsystem, new DrivePose2d()));
    new JoystickButton(rightJoystick, 3).onTrue(new OverideCmd(swerveSubsystem));
    // reseting button for IMU. Usefull for change field orentation forward direction
    new JoystickButton(rightJoystick, 4)
        .onTrue(new InstantCommand(swerveSubsystem::zeroHeading));
    new JoystickButton(rightJoystick, 5)
        .onTrue(new InstantCommand(() -> swerveSubsystem.resetOdometryFromPositivePos(new PosPose2d())));
  }

  public Command getAutonomousCommand() {
    swerveSubsystem.zeroHeading();
    m_autoSelected = m_chooser.getSelected();

    if (m_autoSelected == defaultAuto)
      return commandSequences.defualtAuto(swerveSubsystem);

    return null;
  }
}
