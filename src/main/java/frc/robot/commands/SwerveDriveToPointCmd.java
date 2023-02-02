package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveToPointCmd extends CommandBase {
  SwerveSubsystem swerveSubsystem; 

  PIDController xController, yController, thetaController;
  SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  Pose2d targetPosition;

  public SwerveDriveToPointCmd(SwerveSubsystem m_SwerveSubsystem, Pose2d targetPosition) {
    this.swerveSubsystem = m_SwerveSubsystem;
    this.targetPosition = targetPosition;

    xController = new PIDController(AutoConstants.kTargetPositionPXController, 0, 0);
    yController = new PIDController(AutoConstants.kTargetPositionPYController, 0, 0);
    thetaController = new PIDController(AutoConstants.kTargetPositionPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    this.xLimiter = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    this.yLimiter = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    this.turningLimiter = new SlewRateLimiter(AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared);
    
    addRequirements(m_SwerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xSpeed = xController.calculate(swerveSubsystem.getPose().getX(), targetPosition.getX()) / AutoConstants.kMaxSpeedMetersPerSecond;
    double ySpeed = yController.calculate(swerveSubsystem.getPose().getY(), targetPosition.getY()) / AutoConstants.kMaxSpeedMetersPerSecond;
    double turningSpeed = thetaController.calculate(swerveSubsystem.getPose().getRotation().getRadians(), targetPosition.getRotation().getRadians()) / AutoConstants.kMaxAngularSpeedRadiansPerSecond;

    xSpeed = xLimiter.calculate(xSpeed) * AutoConstants.kMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * AutoConstants.kMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed) * AutoConstants.kMaxAngularSpeedRadiansPerSecond;

    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;

    //use this function if you overide the command to finsih it
    //return swerveSubsystem.getPose().getTranslation().getDistance(targetPosition.getTranslation()) <= AutoConstants.kTargetPositonAcceptableError;
  }
}
