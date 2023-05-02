package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveRotateToAngle extends CommandBase {
  private SwerveSubsystem swerveSubsystem;
  private Rotation2d targetAngle;
  private Timer timer, overide;

  public SwerveRotateToAngle(SwerveSubsystem m_SwerveSubsystem, Rotation2d targetAngle) {
    this.swerveSubsystem = m_SwerveSubsystem;
    this.targetAngle = targetAngle;
    
    timer = new Timer();
    overide = new Timer();

    addRequirements(m_SwerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.initializeRotateToAngle();
    timer.restart();
    overide.restart();
  }

  @Override
  public void execute() {
    swerveSubsystem.excuteRotateToAngle(targetAngle);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    if (overide.hasElapsed(3))
      return true;
    
    // use this function if you overide the command to finsih it
    if ( swerveSubsystem.isAtAngle(targetAngle))
      return timer.hasElapsed(.2);
    timer.restart();
    return false;
  }
}
