package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class OverideCmd extends CommandBase {
  private SwerveSubsystem swerveSubsystem;


  public OverideCmd(SwerveSubsystem m_SwerveSubsystem) {
    this.swerveSubsystem = m_SwerveSubsystem;
    
    addRequirements(m_SwerveSubsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
