package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class BalanceOnBoardCmd extends CommandBase {

  private SwerveSubsystem swerveSubsystem;

  private double currentAngle;
  private double error;
  private double driveSpeed;

  private PIDController pidController = new PIDController(DriveConstants.kBoardBalancedDriveKP, 0, DriveConstants.kBoardBalancedDriveKD);

  /** Command to use Gyro data to resist the tip angle from the board - to stabalize and balanace */
  public BalanceOnBoardCmd(SwerveSubsystem m_SwerveSubsystem) {
    this.swerveSubsystem = m_SwerveSubsystem;
    addRequirements(m_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
    // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
    this.currentAngle = swerveSubsystem.getRoll();

    error = DriveConstants.kBoardBalancedGoalDegrees - currentAngle;
    driveSpeed = -DriveConstants.kPhysicalMaxSpeedMetersPerSecond*pidController.calculate(error, DriveConstants.kBoardBalancedGoalDegrees);
    //Code above does code below, but better.
    //error = DriveConstants.kBoardBalancedGoalDegrees - currentAngle;
    //drivePower = -Math.min(DriveConstants.kBoardBalancedDriveKP * error, 1);

    // Limit the max power
    if (Math.abs(driveSpeed) > DriveConstants.kBoardBalancedSpeed) {
      driveSpeed = Math.copySign(DriveConstants.kBoardBalancedSpeed, driveSpeed);
    }

    swerveSubsystem.driveDirection(driveSpeed, swerveSubsystem.getHeading());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < DriveConstants.kBoardBalancedAngleThresholdDegrees; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
  }
}