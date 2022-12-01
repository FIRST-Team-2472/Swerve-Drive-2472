// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private NetworkTableEntry shuffleEncoder, shuffleTurnEncoder, shuffleAbsoluteEncoder,
    shuffleEncoderSpeed, shuffleTurnEncoderSpeed;

  private TalonFX driveMotor = new TalonFX(DriveConstants.kDriveMotorPort);
  private TalonFX turningMotor = new TalonFX(DriveConstants.kTurningMotorPort);

  private SwerveEncoder abosluteEncoder = new SwerveEncoder(DriveConstants.kDriveAbsoluteEncoderPort);

  private Joystick joystick = new Joystick(0);

  @Override
  public void robotInit() {

    ShuffleboardTab programmerBoard = Shuffleboard.getTab("Programmer Board");

    shuffleEncoder = programmerBoard.add("Drive Count", 0).getEntry();
    shuffleTurnEncoder = programmerBoard.add("Turning Count", 0).getEntry();
    shuffleAbsoluteEncoder = programmerBoard.add("Absolute Value", 0).getEntry();
    shuffleEncoderSpeed = programmerBoard.add("Drive Speed", 0).getEntry();
    shuffleTurnEncoderSpeed = programmerBoard.add("Turning Speed", 0).getEntry();

  }

  @Override
  public void robotPeriodic() {
    shuffleEncoder.setNumber(driveMotor.getSelectedSensorPosition()*ModuleConstants.kDriveEncoderRot2Meter);
    shuffleTurnEncoder.setNumber(turningMotor.getSelectedSensorPosition()*ModuleConstants.kTurningEncoderRot2Rad);
    shuffleAbsoluteEncoder.setNumber(abosluteEncoder.getPosition());

    shuffleEncoderSpeed.setNumber(driveMotor.getSelectedSensorVelocity()*ModuleConstants.kDriveEncoderRPMS2MeterPerSec);
    shuffleTurnEncoderSpeed.setNumber(turningMotor.getSelectedSensorVelocity()*ModuleConstants.kTurningEncoderRPMS2RadPerSec);
  }

  @Override
  public void autonomousInit() {
    
    driveMotor.setSelectedSensorPosition(0);
    turningMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
    driveMotor.setSelectedSensorPosition(0);
    turningMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void teleopPeriodic() {
    turningMotor.set(ControlMode.PercentOutput, 1);
    //driveMotor.set(ControlMode.PercentOutput, joystick.getY());
    //turningMotor.set(ControlMode.PercentOutput, joystick.getX());
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
