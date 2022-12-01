package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.94);
        public static final double kDriveMotorGearRatio = 1/8.14;
        public static final double kTurningMotorGearRatio = 1/12.8;
        //2048 is pulses per rotation of the motor
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters*(1.0/2048.0);
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI*(1.0/2048.0);
        
        public static final double kDriveEncoderRPMS2MeterPerSec = kDriveEncoderRot2Meter*10;
        public static final double kTurningEncoderRPMS2RadPerSec = kTurningEncoderRot2Rad*10;
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(3.75);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(30);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kDriveMotorPort = 11;

        public static final int kTurningMotorPort = 10;

        //Positive is counterclockwise
        public static final boolean kTurningEncoderReversed = false;

        public static final boolean kDriveEncoderReversed = false;

        public static final int kDriveAbsoluteEncoderPort = 21;

        //Positive is counterclockwise
        public static final boolean kDriveAbsoluteEncoderReversed = false;

        public static final double kDriveAbsoluteEncoderOffsetRad = 2.1;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 3.5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 7 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.05;
    }

    public static final class SensorConstants {
        public static final int kPigeonID = 0;
    }
}
