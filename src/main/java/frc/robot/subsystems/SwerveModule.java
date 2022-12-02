package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveEncoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;

    private final SwerveEncoder absoluteEncoder;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        absoluteEncoder = new SwerveEncoder(absoluteEncoderId, absoluteEncoderOffset, absoluteEncoderReversed);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition()*ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        return turningMotor.getSelectedSensorPosition()*ModuleConstants.kTurningEncoderRot2Rad;
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity()*ModuleConstants.kDriveEncoderRPMS2MeterPerSec;
    }

    public double getTurningVelocity() {
        return driveMotor.getSelectedSensorVelocity()*ModuleConstants.kTurningEncoderRPMS2RadPerSec;
    }

    public double getAbsoluteEncoderRad() {
        return absoluteEncoder.getPosition();
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond/ModuleConstants.kTeleDriveMaxSpeedMetersPerSecond);
        turningMotor.set(ControlMode.PercentOutput, (turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getRadians()));
        //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(ControlMode.Velocity, 0);
        turningMotor.set(ControlMode.Velocity, 0);
    }
}
