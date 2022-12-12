// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveModule;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private NetworkTableEntry shuffleEncoder, shuffleTurnEncoder, shuffleAbsoluteEncoder, velocityFB, rotationalVelocityFB;

  private SwerveModule swerveModule = new SwerveModule(DriveConstants.kDriveMotorPort, DriveConstants.kTurningMotorPort, DriveConstants.kDriveEncoderReversed, DriveConstants.kTurningEncoderReversed,
   DriveConstants.kDriveAbsoluteEncoderPort, DriveConstants.kDriveAbsoluteEncoderOffsetRad, DriveConstants.kDriveAbsoluteEncoderReversed);

  private Joystick joystick = new Joystick(0);

  @Override
  public void robotInit() {

    ShuffleboardTab programmerBoard = Shuffleboard.getTab("Programmer Board");

    shuffleEncoder = programmerBoard.add("Drive Count", 0).getEntry();
    shuffleTurnEncoder = programmerBoard.add("Turning Count", 0).getEntry();
    shuffleAbsoluteEncoder = programmerBoard.add("Absolute Value", 0).getEntry();

    velocityFB = programmerBoard.add("Drive Veloctiy: ", 0).getEntry();
    rotationalVelocityFB = programmerBoard.add("Angular Velocity: ", 0).getEntry();
  }

  @Override
  public void robotPeriodic() {
    shuffleEncoder.setNumber(swerveModule.getDrivePosition());
    shuffleTurnEncoder.setNumber(swerveModule.getTurningPosition());
    shuffleAbsoluteEncoder.setNumber(swerveModule.getAbsolutePosition());

    velocityFB.setNumber(swerveModule.getDriveVelocity());
    rotationalVelocityFB.setNumber(swerveModule.getTurningVelocity());
  }

  @Override
  public void autonomousInit() {
    swerveModule.resetEncoders();
  }

  @Override
  public void autonomousPeriodic() {
    swerveModule.setDesiredState(new SwerveModuleState(2.5, new Rotation2d(0)));
  }

  @Override
  public void teleopInit() {
    swerveModule.resetEncoders();
  }

  @Override
  public void teleopPeriodic() {
    double squareX = joystick.getX();
    double squareY = joystick.getY()*-1;

    double x = squareX*Math.sqrt(1-.5*Math.pow(squareY, 2));
    double y = squareY*Math.sqrt(1-.5*Math.pow(squareX, 2));

    if(Math.abs(x) < .1)
      x = 0;
    if(Math.abs(y) < .1)
      y = 0;
    
    double angle = Math.atan2(y, x);
    if (angle >= -Math.PI/2.0)
      angle-=(Math.PI/2.0);
    else
      angle= Math.PI +(angle/2.0);
    
    swerveModule.setDesiredState(new SwerveModuleState(.25*DriveConstants.kPhysicalMaxSpeedMetersPerSecond*Math.hypot(x, y), new Rotation2d(angle)));
    System.out.println(angle);
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
