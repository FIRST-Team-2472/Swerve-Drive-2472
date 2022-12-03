package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kDriveMotorPort, 
            DriveConstants.kTurningMotorPort, 
            DriveConstants.kDriveEncoderReversed, 
            DriveConstants.kTurningEncoderReversed,
            DriveConstants.kDriveAbsoluteEncoderPort, 
            DriveConstants.kDriveAbsoluteEncoderOffsetRad, 
            DriveConstants.kDriveAbsoluteEncoderReversed);


    private final PigeonIMU gyro = new PigeonIMU(SensorConstants.kPigeonID);

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.configFactoryDefault();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getPitch(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void stopModules() {
        frontLeft.stop();

    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        //SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        /*frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);*/
    }
}
