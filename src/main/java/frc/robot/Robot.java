// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
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
  private NetworkTableEntry driveS, turningS, absoluteS, driveVelocityS, turningVelocityS;

  private TalonFX driveMotor = new TalonFX(DriveConstants.kDriveMotorPort);
  private TalonFX turningMotor = new TalonFX(DriveConstants.kTurningMotorPort);
  private SwerveEncoder abosluteEncoder = new SwerveEncoder(DriveConstants.kDriveAbsoluteEncoderPort);

  @Override
  public void robotInit() {

    ShuffleboardTab programmerBoard = Shuffleboard.getTab("Programmer Board");

    driveS = programmerBoard.add("Drive Count", 0).getEntry();
    turningS = programmerBoard.add("Turning Count", 0).getEntry();
    absoluteS = programmerBoard.add("Absolute Value", 0).getEntry();

    driveVelocityS = programmerBoard.add("Drive Velocity", 0).getEntry();
    turningVelocityS = programmerBoard.add("Turning Velocity", 0).getEntry();
  }

  @Override
  public void robotPeriodic() {
    driveS.setNumber(driveMotor.getSelectedSensorPosition()*ModuleConstants.kDriveEncoderRot2Meter);
    turningS.setNumber(turningMotor.getSelectedSensorPosition()*ModuleConstants.kTurningEncoderRot2Rad);
    absoluteS.setNumber(abosluteEncoder.getPosition());

    driveVelocityS.setNumber(driveMotor.getSelectedSensorVelocity()*ModuleConstants.kDriveEncoderRPMS2MeterPerSec);
    turningVelocityS.setNumber(turningMotor.getSelectedSensorVelocity()*ModuleConstants.kTurningEncoderRPMS2RadPerSec);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
    driveMotor.setSelectedSensorPosition(0);
    turningMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void teleopPeriodic() {
    turningMotor.set(ControlMode.PercentOutput, .2);
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
