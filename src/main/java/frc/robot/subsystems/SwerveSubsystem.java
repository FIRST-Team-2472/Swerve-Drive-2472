package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final PigeonIMU gyro = new PigeonIMU(SensorConstants.kPigeonID);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0),getModulePositions());
    private GenericEntry headingShuffleBoard, odometerShuffleBoard, trueAngleShuffleBoard;

    public SwerveSubsystem() {
        ShuffleboardTab programmerBoard = Shuffleboard.getTab("Programmer Board");
       
        headingShuffleBoard = programmerBoard.add("Robot Heading", 0).getEntry();
        odometerShuffleBoard = programmerBoard.add("Robot Location", "").getEntry();
        trueAngleShuffleBoard = programmerBoard.add("Robot Angle", 0).getEntry();


        //zeros heading after pigeon boots up
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.setYaw(0);
    }

    public double getHeading() {
        //imu is backwards, so it is multiplied by negative one
        return -Math.IEEEremainder(gyro.getYaw(), 360);
    }

    public double getRoll() {
        return -gyro.getRoll();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getTrueAngle() {
        //Gets the true angle of the balancing platform if robot is at an angle on the platform.
        if ((getHeading() >= -45 && getHeading() <= 45) || getHeading() <= -135 || getHeading() >= 135) 
            //Calculation for balancing near 0, 180, and -180 but is inaccurate at 90 and -90 degrees of Yaw.
            return (180/Math.PI)*Math.atan((1/(Math.cos(getHeading()/180*Math.PI)))*Math.tan(getRoll()/180*Math.PI));
        //Alternate calculation for balancing near 90 and -90 but is inaccurate at 0, 180, and -180 degrees of Yaw.
        return (180/Math.PI)*Math.atan((1/(Math.cos(getHeading()/180*Math.PI+Math.PI/2)))*Math.tan(getPitch()/180*Math.PI));
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        //this method comes from the subsystem class we inherited. Runs constantly while robot is on

        //changes heading and module states into an x,y corrordanite
        odometer.update(getRotation2d(), getModulePositions());
        headingShuffleBoard.setDouble(getHeading());
        odometerShuffleBoard.setString(getPose().getTranslation().toString());

        trueAngleShuffleBoard.setDouble(getTrueAngle());
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] temp = {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()};
        return temp;
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        //if their speed is larger then the physical max speed, it reduces all speeds until they are smaller than physical max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        //sets the modules to desired states
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void driveDirectionForward(double speed) {
        SwerveModuleState[] desiredStates = new SwerveModuleState[4];

        for (int i = 0; i < 4; i++) {
            desiredStates[i] = new SwerveModuleState(speed, Rotation2d.fromDegrees(-getHeading()));
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        //sets the modules to desired states
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }
}
