package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;

    private final PIDController turningPidController;

    private final SwerveEncoder absoluteEncoder;

    private NetworkTableEntry desiredSpeed, desiredDir;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        absoluteEncoder = new SwerveEncoder(absoluteEncoderId, absoluteEncoderOffset, absoluteEncoderReversed);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        ShuffleboardTab programmerBoard = Shuffleboard.getTab("Programmer Board");

        desiredSpeed = programmerBoard.add("Desired Speed:", 0).getEntry();
        desiredDir = programmerBoard.add("Desired Direction:", 0).getEntry();


        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition()*ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        //Use absolute encoder for most things instead of this
        //Isn't currently bound to a certian range. Will count up indefintly

        //measured in revolutions not radians. easier to understand
        return (turningMotor.getSelectedSensorPosition()*ModuleConstants.kTurningEncoderRot2Rad)/(2*Math.PI);
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity()*ModuleConstants.kDriveEncoderRPMS2MeterPerSec;
    }

    public double getTurningVelocity() {
        //measured in revolutions not radians. easier to understand
        return (driveMotor.getSelectedSensorVelocity()*ModuleConstants.kTurningEncoderRPMS2RadPerSec)/(2*Math.PI);
    }

    public double getAbsolutePosition() {
        return absoluteEncoder.getPosition();
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getAbsolutePosition()));
    }

    // a swerve module state is composed of a speed and direction
    public void setDesiredState(SwerveModuleState state) {
        //prevents wheels from changing direction if it is given barely any speed
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(ControlMode.PercentOutput, turningPidController.calculate(getAbsolutePosition(), state.angle.getRadians()));

        desiredSpeed.setNumber(state.speedMetersPerSecond/DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        desiredDir.setNumber(turningPidController.calculate(getAbsolutePosition(), state.angle.getRadians()));
    }

    public double getPID(double angle) {
        return turningPidController.calculate(getAbsolutePosition(), angle);
    }

    public void stop() {
        driveMotor.set(ControlMode.PercentOutput, 0);
        turningMotor.set(ControlMode.PercentOutput, 0);
    }
}
