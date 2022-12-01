// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.SwerveModule;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private NetworkTableEntry shuffleEncoder, shuffleTurnEncoder, shuffleAbsoluteEncoder;
  private SwerveModule wheel1 = new SwerveModule(DriveConstants.kDriveMotorPort, DriveConstants.kTurningMotorPort, DriveConstants.kDriveEncoderReversed, DriveConstants.kTurningEncoderReversed,
   DriveConstants.kDriveAbsoluteEncoderPort, DriveConstants.kDriveAbsoluteEncoderOffsetRad, DriveConstants.kDriveAbsoluteEncoderReversed);
  /*private TalonFX driveMotor = new TalonFX(DriveConstants.kDriveMotorPort);
  private TalonFX turningMotor = new TalonFX(DriveConstants.kTurningMotorPort);

  private SwerveEncoder abosluteEncoder = new SwerveEncoder(DriveConstants.kDriveAbsoluteEncoderPort, DriveConstants.kDriveAbsoluteEncoderOffsetRad);
  */
  private Joystick joystick = new Joystick(0);

  @Override
  public void robotInit() {

    ShuffleboardTab programmerBoard = Shuffleboard.getTab("Programmer Board");

    shuffleEncoder = programmerBoard.add("Drive Count", 0).getEntry();
    shuffleTurnEncoder = programmerBoard.add("Turning Count", 0).getEntry();
    shuffleAbsoluteEncoder = programmerBoard.add("Absolute Value", 0).getEntry();

  }

  @Override
  public void robotPeriodic() {
    shuffleEncoder.setNumber(wheel1.getDrivePosition());
    shuffleTurnEncoder.setNumber(wheel1.getTurningPosition());
    //shuffleAbsoluteEncoder.setNumber(abosluteEncoder.getPosition());
  }

  @Override
  public void autonomousInit() {
    wheel1.resetEncoders();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    wheel1.resetEncoders();
  }

  @Override
  public void teleopPeriodic() {
    wheel1.setDesiredState(new SwerveModuleState(3.5, new Rotation2d(0)));

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
