// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private GenericEntry FLdriveS, FLturningS, FLAbsoluteS,
      BLdriveS, BLturningS, BLAbsoluteS,
      BRdriveS, BRturningS, BRAbsoluteS,
      FRdriveS, FRturningS, FRAbsoluteS,
      FLdriveVelocity, FRdriveVelocity, BLdriveVelocity, BRdriveVelocity,
      FLturningVelocity, FRturningVelocity, BLturningVelocity, BRturningVelocity;

  private TalonFX FLdriveMotor = new TalonFX(DriveConstants.kFrontLeftDriveMotorPort);
  private TalonFX FLturningMotor = new TalonFX(DriveConstants.kFrontLeftTurningMotorPort);
  private SwerveEncoder FLabosluteEncoder = new SwerveEncoder(DriveConstants.kFrontLeftDriveAbsoluteEncoderPort);

  private TalonFX BLdriveMotor = new TalonFX(DriveConstants.kBackLeftDriveMotorPort);
  private TalonFX BLturningMotor = new TalonFX(DriveConstants.kBackLeftTurningMotorPort);
  private SwerveEncoder BLabosluteEncoder = new SwerveEncoder(DriveConstants.kBackLeftDriveAbsoluteEncoderPort);

  private TalonFX BRdriveMotor = new TalonFX(DriveConstants.kBackRightDriveMotorPort);
  private TalonFX BRturningMotor = new TalonFX(DriveConstants.kBackRightTurningMotorPort);
  private SwerveEncoder BRabosluteEncoder = new SwerveEncoder(DriveConstants.kBackRightDriveAbsoluteEncoderPort);

  private TalonFX FRdriveMotor = new TalonFX(DriveConstants.kFrontRightDriveMotorPort);
  private TalonFX FRturningMotor = new TalonFX(DriveConstants.kFrontRightTurningMotorPort);
  private SwerveEncoder FRabosluteEncoder = new SwerveEncoder(DriveConstants.kFrontRightDriveAbsoluteEncoderPort);

  @Override
  public void robotInit() {

    ShuffleboardTab programmerBoard = Shuffleboard.getTab("Programmer Board");
    ShuffleboardTab velocityBoard = Shuffleboard.getTab("Velocity Board");

    FLdriveS = programmerBoard.add("FL Drive Count", 0).getEntry();
    FLturningS = programmerBoard.add("FL Turning Count", 0).getEntry();
    FLAbsoluteS = programmerBoard.add("FL Absolute Value", 0).getEntry();

    BLdriveS = programmerBoard.add("BL Drive Count", 0).getEntry();
    BLturningS = programmerBoard.add("BL Turning Count", 0).getEntry();
    BLAbsoluteS = programmerBoard.add("BL Absolute Value", 0).getEntry();

    BRdriveS = programmerBoard.add("BR Drive Count", 0).getEntry();
    BRturningS = programmerBoard.add("BR Turning Count", 0).getEntry();
    BRAbsoluteS = programmerBoard.add("BR Absolute Value", 0).getEntry();

    FRdriveS = programmerBoard.add("FR Drive Count", 0).getEntry();
    FRturningS = programmerBoard.add("FR Turning Count", 0).getEntry();
    FRAbsoluteS = programmerBoard.add("FR Absolute Value", 0).getEntry();

    FLdriveVelocity = velocityBoard.add("FL Drive Vel", 0).getEntry();
    FLturningVelocity = velocityBoard.add("FL Turning Vel", 0).getEntry();

    FRdriveVelocity = velocityBoard.add("FR Drive Vel", 0).getEntry();
    FRturningVelocity = velocityBoard.add("FR Turning Vel", 0).getEntry();
    
    BLdriveVelocity = velocityBoard.add("BL Drive Vel", 0).getEntry();
    BLturningVelocity = velocityBoard.add("BL Turning Vel", 0).getEntry();
    
    BRdriveVelocity = velocityBoard.add("BR Drive Vel", 0).getEntry();
    BRturningVelocity = velocityBoard.add("BR Turning Vel", 0).getEntry();
  }

  @Override
  public void robotPeriodic() {
    FLdriveS.setDouble(FLdriveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveEncoderRot2Meter);
    FLturningS.setDouble(FLturningMotor.getSelectedSensorPosition() * ModuleConstants.kTurningEncoderRot2Rad);
    FLAbsoluteS.setDouble(FLabosluteEncoder.getPosition());

    BLdriveS.setDouble(BLdriveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveEncoderRot2Meter);
    BLturningS.setDouble(BLturningMotor.getSelectedSensorPosition() * ModuleConstants.kTurningEncoderRot2Rad);
    BLAbsoluteS.setDouble(BLabosluteEncoder.getPosition());

    BRdriveS.setDouble(BRdriveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveEncoderRot2Meter);
    BRturningS.setDouble(BRturningMotor.getSelectedSensorPosition() * ModuleConstants.kTurningEncoderRot2Rad);
    BRAbsoluteS.setDouble(BRabosluteEncoder.getPosition());

    FRdriveS.setDouble(FRdriveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveEncoderRot2Meter);
    FRturningS.setDouble(FRturningMotor.getSelectedSensorPosition() * ModuleConstants.kTurningEncoderRot2Rad);
    FRAbsoluteS.setDouble(FRabosluteEncoder.getPosition());

    FLdriveVelocity.setDouble(FLdriveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderRPMS2MeterPerSec);
    FLturningVelocity.setDouble(FLturningMotor.getSelectedSensorVelocity() * ModuleConstants.kTurningEncoderRPMS2RadPerSec);
    
    FRdriveVelocity.setDouble(FRdriveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderRPMS2MeterPerSec);
    FRturningVelocity.setDouble(FRturningMotor.getSelectedSensorVelocity() * ModuleConstants.kTurningEncoderRPMS2RadPerSec);
    
    BLdriveVelocity.setDouble(BLdriveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderRPMS2MeterPerSec);
    BLturningVelocity.setDouble(BLturningMotor.getSelectedSensorVelocity() * ModuleConstants.kTurningEncoderRPMS2RadPerSec);
    
    BRdriveVelocity.setDouble(BRdriveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderRPMS2MeterPerSec);
    BRturningVelocity.setDouble(BRturningMotor.getSelectedSensorVelocity() * ModuleConstants.kTurningEncoderRPMS2RadPerSec);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

    FLdriveMotor.setSelectedSensorPosition(0);
    FLturningMotor.setSelectedSensorPosition(0);

    BLdriveMotor.setSelectedSensorPosition(0);
    BLturningMotor.setSelectedSensorPosition(0);

    BRdriveMotor.setSelectedSensorPosition(0);
    BRturningMotor.setSelectedSensorPosition(0);

    FRdriveMotor.setSelectedSensorPosition(0);
    FRturningMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
    FLdriveMotor.setSelectedSensorPosition(0);
    FLturningMotor.setSelectedSensorPosition(0);

    BLdriveMotor.setSelectedSensorPosition(0);
    BLturningMotor.setSelectedSensorPosition(0);

    BRdriveMotor.setSelectedSensorPosition(0);
    BRturningMotor.setSelectedSensorPosition(0);

    FRdriveMotor.setSelectedSensorPosition(0);
    FRturningMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void testPeriodic() {
    /* 
    FRdriveMotor.set(ControlMode.PercentOutput, 1);
    BRdriveMotor.set(ControlMode.PercentOutput, 1);
    FLdriveMotor.set(ControlMode.PercentOutput, 1);
    BLdriveMotor.set(ControlMode.PercentOutput, 1);
   */
    
    FRturningMotor.set(ControlMode.PercentOutput, 1);
    BRturningMotor.set(ControlMode.PercentOutput, 1);
    FLturningMotor.set(ControlMode.PercentOutput, 1);
    BLturningMotor.set(ControlMode.PercentOutput, 1);
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
