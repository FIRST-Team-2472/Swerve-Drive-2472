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
  private NetworkTableEntry FLdriveS, FLturningS, FLAbsoluteS,
    BLdriveS, BLturningS, BLAbsoluteS,
    BRdriveS, BRturningS, BRAbsoluteS,
    FRdriveS, FRturningS, FRAbsoluteS;

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
  }

  @Override
  public void robotPeriodic() {
    FLdriveS.setNumber(FLdriveMotor.getSelectedSensorPosition()*ModuleConstants.kDriveEncoderRot2Meter);
    FLturningS.setNumber(FLturningMotor.getSelectedSensorPosition()*ModuleConstants.kTurningEncoderRot2Rad);
    FLAbsoluteS.setNumber(FLabosluteEncoder.getPosition());

    BLdriveS.setNumber(BLdriveMotor.getSelectedSensorPosition()*ModuleConstants.kDriveEncoderRot2Meter);
    BLturningS.setNumber(BLturningMotor.getSelectedSensorPosition()*ModuleConstants.kTurningEncoderRot2Rad);
    BLAbsoluteS.setNumber(BLabosluteEncoder.getPosition());

    BRdriveS.setNumber(BRdriveMotor.getSelectedSensorPosition()*ModuleConstants.kDriveEncoderRot2Meter);
    BRturningS.setNumber(BRturningMotor.getSelectedSensorPosition()*ModuleConstants.kTurningEncoderRot2Rad);
    BRAbsoluteS.setNumber(BRabosluteEncoder.getPosition());

    FRdriveS.setNumber(FRdriveMotor.getSelectedSensorPosition()*ModuleConstants.kDriveEncoderRot2Meter);
    FRturningS.setNumber(FRturningMotor.getSelectedSensorPosition()*ModuleConstants.kTurningEncoderRot2Rad);
    FRAbsoluteS.setNumber(FRabosluteEncoder.getPosition());
  }

  @Override
  public void autonomousInit() {
    
    FLdriveMotor.setSelectedSensorPosition(0);
    FLturningMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
    FLdriveMotor.setSelectedSensorPosition(0);
    FLturningMotor.setSelectedSensorPosition(0);
  }

  @Override
  public void teleopPeriodic() {

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
