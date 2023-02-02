package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class BalanceOnBoardCmd extends CommandBase {

  private SwerveSubsystem swerveSubsystem;

  private double driveSpeed, timer;

  private PIDController pidController = new PIDController(DriveConstants.kBoardBalancedDriveKP, 0, DriveConstants.kBoardBalancedDriveKD);

  /** Command to use Gyro data to resist the tip angle from the board - to stabalize and balanace */
  public BalanceOnBoardCmd(SwerveSubsystem m_SwerveSubsystem) {
    this.swerveSubsystem = m_SwerveSubsystem;
    addRequirements(m_SwerveSubsystem);

    timer = DriveConstants.balanceCounter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSpeed = DriveConstants.kPhysicalMaxSpeedMetersPerSecond*pidController.calculate(-swerveSubsystem.getTrueAngle(), DriveConstants.kBoardBalancedGoalDegrees);

    swerveSubsystem.driveDirectionForward(driveSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(swerveSubsystem.getTrueAngle()) < DriveConstants.kBoardBalancedAngleThresholdDegrees) {
      timer--;
      return timer <= 0; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
    }
    timer = DriveConstants.balanceCounter;
    return false;
  }
}