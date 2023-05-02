package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;

    // Abslute encoder for swerve drive module
    private CANCoder absoluteEncoder;
    private double absoluteEncoderOffset;
    private boolean absoluteEncoderReversed;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        absoluteEncoder = new CANCoder(absoluteEncoderId);
        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        // lets us use a PID system for the turning motor
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        // tells the PID controller that our motor can go from -PI to PI (it can rotate
        // continously)
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        // Use absolute encoder for most things instead of this
        // Isn't currently bound to a certian range. Will count up indefintly

        // measured in revolutions not radians. easier to understand
        return (turningMotor.getSelectedSensorPosition() * ModuleConstants.kTurningEncoderRot2Rad) / (2 * Math.PI);
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderRPMS2MeterPerSec;
    }

    public double getTurningVelocity() {
        // measured in revolutions not radians. easier to understand
        return (driveMotor.getSelectedSensorVelocity() * ModuleConstants.kTurningEncoderRPMS2RadPerSec) / (2 * Math.PI);
    }

    public double getAbsolutePosition() {
        // converts from 0-360 to -PI to PI then applies abosluteEncoder offset and
        // reverse
        double angle = Units.degreesToRadians(absoluteEncoder.getAbsolutePosition());
        angle -= absoluteEncoderOffset;
        angle *= absoluteEncoderReversed ? -1 : 1;

        // atan2 funtion range in -PI to PI, so it automaticaly converts (needs the sin
        // and cos to) any input angle to that range
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsolutePosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getAbsolutePosition()));
    }

    // a swerve module state is composed of a speed and direction
    public void setDesiredState(SwerveModuleState state) {
        // prevents wheels from changing direction if it is given barely any speed
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // make the swerve module doesn't ever turn more than 90 degrees instead of 180;
        state = SwerveModuleState.optimize(state, getState().angle);
        // set the motor drive speed to the speed given in swerveModuleState
        driveMotor.set(ControlMode.PercentOutput,
                state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // uses PID to slow down as it approaches the target ange given in
        // swerveModuleState
        turningMotor.set(ControlMode.PercentOutput,
                turningPidController.calculate(getAbsolutePosition(), state.angle.getRadians()));
    }

    // a swerve module state is composed of a speed and direction
    public void jankSetDesiredStateForExtremeGamers(SwerveModuleState state) {

        // make the swerve module doesn't ever turn more than 90 degrees instead of 180;
        state = SwerveModuleState.optimize(state, getState().angle);
        // uses PID to slow down as it approaches the target ange given in
        // swerveModuleState
        turningMotor.set(ControlMode.PercentOutput,
                turningPidController.calculate(getAbsolutePosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }
}
