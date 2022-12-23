package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.94);
        //gear ratio is inversed (1/gear ratio) so we can multiply instead of dividing the Rot to meter funtion 
        public static final double kDriveMotorGearRatio = 1/8.14;
        public static final double kTurningMotorGearRatio = 1/12.8;
        //2048 is pulses per rotation of the motor
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters*(1.0/2048.0);
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI*(1.0/2048.0);
        
        //CTRE mesures their velcity in 100ms, so we multiply it by 10 to get 1s
        public static final double kDriveEncoderRPMS2MeterPerSec = kDriveEncoderRot2Meter*10;
        public static final double kTurningEncoderRPMS2RadPerSec = kTurningEncoderRot2Rad*10;
        //use guess and check to find. when the module is overshooting this needs to be fine tuned
        public static final double kPTurning = 0.2;
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(18);
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(30);
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 11;
        public static final int kBackLeftDriveMotorPort = 13;
        public static final int kFrontRightDriveMotorPort = 17;
        public static final int kBackRightDriveMotorPort = 15;

        public static final int kFrontLeftTurningMotorPort = 10;
        public static final int kBackLeftTurningMotorPort = 12;
        public static final int kFrontRightTurningMotorPort = 16;
        public static final int kBackRightTurningMotorPort = 14;

        //Positive should be clockwise
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 21;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 22;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 24;
        public static final int kBackRightDriveAbsoluteEncoderPort = 23;

        //Positive should be clockwise
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        //To find set the motors forward record the value (don't inverse the value)
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 2.00338;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 1.92515;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -3.0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -0.43718;

        //To find set the modules to 100% and see what speed cap out at
        public static final double kPhysicalMaxSpeedMetersPerSecond = 4;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 7 * 2 * Math.PI;

        //arbitrary chosen based on what drivers pick
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 3;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 8;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = Math.PI/3;
    }

    public static final class OIConstants {
        //joystick id
        public static final int kDriverControllerPort = 0;

        //axis and buttons id for drive joysticks
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 3;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        //area were joysticks will not activate
        public static final double kDeadband = 0.05;
    }

    public static final class SensorConstants {
        public static final int kPigeonID = 20;
    }
}
