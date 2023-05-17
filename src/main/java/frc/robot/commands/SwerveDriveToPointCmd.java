package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.swerveExtras.DrivePose2d;
import frc.robot.subsystems.swerveExtras.PosPose2d;

public class SwerveDriveToPointCmd extends CommandBase {
  private SwerveSubsystem swerveSubsystem;
  private Pose2d targetPosition;
  private Timer timer;

  public SwerveDriveToPointCmd(SwerveSubsystem m_SwerveSubsystem, DrivePose2d targetPosition) {
    this.swerveSubsystem = m_SwerveSubsystem;
    this.targetPosition = targetPosition;
    
    timer = new Timer();

    addRequirements(m_SwerveSubsystem);
  }

  public SwerveDriveToPointCmd(SwerveSubsystem m_SwerveSubsystem, PosPose2d targetPosition) {
    this.swerveSubsystem = m_SwerveSubsystem;
    this.targetPosition = targetPosition.toDrivePose2d();
    
    timer = new Timer();

    addRequirements(m_SwerveSubsystem);
  }

  @Override
  public void initialize() {
    timer.restart();
  }

  @Override
  public void execute() {
    swerveSubsystem.excuteDriveToPointAndRotate(targetPosition);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    // use this function if you overide the command to finsih it
    if ( swerveSubsystem.isAtPoint(targetPosition.getTranslation()) && swerveSubsystem.isAtAngle(targetPosition.getRotation()))
      return timer.hasElapsed(.05);
    timer.restart();
    return false;
  }
}
