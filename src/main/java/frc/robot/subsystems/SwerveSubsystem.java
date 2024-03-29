package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.Constants.TargetPosConstants;
import frc.robot.Constants.TeleDriveConstants;
import frc.robot.subsystems.swerveExtras.AccelerationLimiter;
import frc.robot.subsystems.swerveExtras.DrivePoint;
import frc.robot.subsystems.swerveExtras.DrivePose2d;
import frc.robot.subsystems.swerveExtras.FieldPose2d;
import frc.robot.subsystems.swerveExtras.PosPose2d;
import frc.robot.subsystems.swerveExtras.PositivePoint;

public class SwerveSubsystem extends SubsystemBase {
    // All of the constants.
    // There are so many constants.
    // Holy jesus balls.
    // -Wes Korba, 2023

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
    // It's still going, oh my god.
    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    private final PigeonIMU gyro = new PigeonIMU(SensorConstants.kPigeonID);
    // Sets the preliminary odometry. This gets refined by the PhotonVision class,
    // but this is the original.
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), getModulePositions());
    private GenericEntry headingShuffleBoard, odometerShuffleBoard, rollSB, pitchSB;
    private static BooleanSubscriber isOnRed;
    private final AccelerationLimiter xLimiter, yLimiter, turningLimiter;
    private PIDController xController, yController, thetaController;

    private static final SendableChooser<String> colorChooser = new SendableChooser<>();
    private final String red = "Red", blue = "Blue";

    public SwerveSubsystem() {
        // Gets tabs from Shuffleboard
        ShuffleboardTab programmerBoard = Shuffleboard.getTab("Programmer Board");
        ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");

        // Sets up the different displays on suffle board
        headingShuffleBoard = programmerBoard.add("Robot Heading", 0).getEntry();
        odometerShuffleBoard = programmerBoard.add("Robot Location", "").getEntry();
        rollSB = programmerBoard.add("Roll", 0).getEntry();
        pitchSB = programmerBoard.add("Pitch", 0).getEntry();

        // Gets the field infomation
        NetworkTable firstInfo = NetworkTableInstance.getDefault().getTable("FMSInfo");
        // Gets the team color from the field information
        isOnRed = firstInfo.getBooleanTopic("IsRedAlliance").subscribe(false);

        // makes a team color choser
        colorChooser.addOption(red, red);
        colorChooser.addOption(blue, blue);
        driverBoard.add("Team Chooser", colorChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

        xLimiter = new AccelerationLimiter(TeleDriveConstants.kMaxAccelerationUnitsPerSecond);
        yLimiter = new AccelerationLimiter(TeleDriveConstants.kMaxAccelerationUnitsPerSecond);
        turningLimiter = new AccelerationLimiter(TeleDriveConstants.kMaxAngularAccelerationUnitsPerSecond);

        xController = new PIDController(TargetPosConstants.kPDriveController, 0, 0);
        yController = new PIDController(TargetPosConstants.kPDriveController, 0, 0);
        thetaController = new PIDController(TargetPosConstants.kPAngleController, 0, 0);

        // zeros heading after pigeon boots up
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    // Just a quick method that zeros the IMU.
    public void zeroHeading() {
        gyro.setYaw(0);
    }

    // Gets the yaw/heading of the robot. getting this right is very important for
    // swerve
    public double getHeading() {
        // imu is backwards, so it is multiplied by negative one
        return -Math.IEEEremainder(gyro.getYaw(), 360);
    }

    // Gets the roll of the robot based on the IMU.
    public double getRoll() {
        return gyro.getRoll();
    }

    // Gets the pitch of the robot based on what the IMU percieves.
    public double getPitch() {
        return -gyro.getPitch();
    }

    // used for anything that requires team color.
    // this is housed in swerve subsystem since it uses it the most
    public static boolean isOnRed() {
        // gets the selected team color from the suffleboard
        String choices = colorChooser.getSelected();
        if (choices == "Red")
            return true;
        if (choices == "Blue")
            return false;
        // if no team selected on suffleboard, it will default to the field info
        return isOnRed.get();
    }

    // gets our current velocity relative to the x of the field
    public double getXSpeedFieldRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.vxMetersPerSecond;
    }

    // gets our current velocity relative to the x of the robot (front/back)
    public double getXSpeedRobotRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());

        return temp.vxMetersPerSecond;
    }

    // gets our current velocity relative to the y of the field
    public double getYSpeedFieldRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.vyMetersPerSecond;
    }

    // gets our current velocity relative to the y of the robot (left/right)
    public double getYSpeedRobotRel() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());

        return temp.vyMetersPerSecond;
    }

    // gets our current angular velocity
    public double getRotationalSpeed() {
        ChassisSpeeds temp = DriveConstants.kDriveKinematics.toChassisSpeeds(frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(), backRight.getState());
        temp = ChassisSpeeds.fromFieldRelativeSpeeds(temp, getRotation2d());

        return temp.omegaRadiansPerSecond;
    }

    // Gets our heading and translates it to Rotation2d
    // (all of swerve methods use Rotation2d)
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // Gets our drive position aka where the odometer thinks we are
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public DrivePoint getDrivePoint() {
        return new DrivePoint(odometer.getPoseMeters().getTranslation());
    }

    public void resetOdometryFromFieldPos(FieldPose2d fieldPos) {
        resetOdometryFromPositivePos(fieldPos.toPosPose2d());
    }

    public void resetOdometryFromPositivePos(PosPose2d posPose) {
        // heading must be zeroed correctly before running this method
        // Resets the odometry based on new information.
        // offset is used if the robot starts in a direction that isn't the forward
        // direction

        DrivePose2d tempPos = posPose.toDrivePose2d();
        odometer.resetPosition(getRotation2d(), getModulePositions(), tempPos);
    }

    public SwerveModulePosition[] getModulePositions() {
        // Finds the position of each individual module based on the encoder values.
        SwerveModulePosition[] temp = { frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
                backRight.getPosition() };
        return temp;
    }


    public void intializeJoystickRunFromField() {
        xLimiter.setLimit(TeleDriveConstants.kMaxAccelerationUnitsPerSecond);
        yLimiter.setLimit(TeleDriveConstants.kMaxAccelerationUnitsPerSecond);
        turningLimiter.setLimit(TeleDriveConstants.kMaxAngularAccelerationUnitsPerSecond);

        xLimiter.reset(getXSpeedFieldRel());
        yLimiter.reset(getYSpeedFieldRel());
        turningLimiter.reset(getRotationalSpeed());
    }

    public void excuteJoystickRunFromField(double xSpeedPercent, double ySpeedPercent, double thetaSpeedPercent) {
        // 3. Make the driving smoother (limits acceleration)
        xSpeedPercent = xLimiter.calculate(xSpeedPercent * TeleDriveConstants.kMaxSpeedMetersPerSecond);
        ySpeedPercent = yLimiter.calculate(ySpeedPercent * TeleDriveConstants.kMaxSpeedMetersPerSecond);
        thetaSpeedPercent = turningLimiter
                .calculate(thetaSpeedPercent * TeleDriveConstants.kMaxAngularSpeedRadiansPerSecond);

        runModulesFieldRelative(xSpeedPercent, ySpeedPercent, thetaSpeedPercent);
    }

    public void initializeDriveToPointAndRotate() {
        xLimiter.setLimit(TargetPosConstants.kForwardMaxAcceleration,
        TargetPosConstants.kBackwardMaxAcceleration);
        yLimiter.setLimit(TargetPosConstants.kForwardMaxAcceleration,
        TargetPosConstants.kBackwardMaxAcceleration);
        xLimiter.reset(getXSpeedFieldRel());
        yLimiter.reset(getYSpeedFieldRel());

        xController.setPID(TargetPosConstants.kPDriveController, 0, 0);
        xController.reset();
        yController.setPID(TargetPosConstants.kPDriveController, 0, 0);
        yController.reset();
        thetaController.setPID(TargetPosConstants.kPAngleController, 0, 0);
        thetaController.reset();
    }

    public void excuteDriveToPointAndRotate(Pose2d targetPosition) {
        double xSpeed = MathUtil.clamp(
                xController.calculate(getPose().getX(), targetPosition.getX()), -1, 1);
        double ySpeed = MathUtil.clamp(
                yController.calculate(getPose().getY(), targetPosition.getY()), -1, 1);

        Rotation2d angleDifference = getRotation2d().minus(targetPosition.getRotation());
        double turningSpeed = MathUtil.clamp(thetaController.calculate(angleDifference.getRadians(),
                0), -1, 1);
        turningSpeed *= TargetPosConstants.kMaxAngularSpeed;
        turningSpeed += Math.copySign(TargetPosConstants.kMinAngluarSpeedRadians, turningSpeed);

        xSpeed = xLimiter.calculate(xSpeed * TargetPosConstants.kMaxSpeedMetersPerSecond);
        ySpeed = yLimiter.calculate(ySpeed * TargetPosConstants.kMaxSpeedMetersPerSecond);

        double unitCircleAngle = Math.atan2(ySpeed, xSpeed);
        xSpeed += Math.copySign(TargetPosConstants.kMinSpeedMetersPerSec, xSpeed) * Math.abs(Math.cos(unitCircleAngle));
        ySpeed += Math.copySign(TargetPosConstants.kMinSpeedMetersPerSec, ySpeed) * Math.abs(Math.sin(unitCircleAngle));

        runModulesFieldRelative(xSpeed, ySpeed, turningSpeed);
    }

    public void initializeRotateToAngle() {
        thetaController.setPID(TargetPosConstants.kPAngleController, 0, 0);
        thetaController.reset();
    }

    public void excuteRotateToAngle(Rotation2d targetPosition) {
        Rotation2d angleDifference = getRotation2d().minus(targetPosition);
        double turningSpeed = MathUtil.clamp(thetaController.calculate(angleDifference.getRadians(),
                0), -1, 1) * TargetPosConstants.kMaxAngularSpeed;

        turningSpeed += Math.copySign(TargetPosConstants.kMinAngluarSpeedRadians, turningSpeed);

        runModulesFieldRelative(0, 0, turningSpeed);
    }

    private void runModulesFieldRelative(double xSpeed, double ySpeed, double turningSpeed) {
        // Converts robot speeds to speeds relative to field
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, getRotation2d());

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Output each module states to wheels
        setModuleStates(moduleStates);
    }

    private void runModulesRobotRelative(double xSpeed, double ySpeed, double turningSpeed) {
        // Makes the speeds usable for kinematics
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // Output each module states to wheels
        setModuleStates(moduleStates);
    }

    public void stopModules() {
        // Stops all of the modules. Use in emergencies.
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }


    public boolean isAtPoint(Translation2d targetDrivePos) {
        return getPose().getTranslation().getDistance(targetDrivePos) //
                <= TargetPosConstants.kAcceptableDistanceError; //
    }

    public boolean isAtAngle(Rotation2d angle) {
        return Math.abs(getRotation2d().minus(angle).getDegrees()) //
                <= TargetPosConstants.kAcceptableAngleError;
    }

    public Pose2d getNearestPosFromArray(Pose2d[] comparisonArray) {
        // Comparison Array should be composed of positive points
        // Finds the coord nearest to the robot from the array.
        // No idea how Sawyer figured this out, but he did.

        // Covernts odometry pos (a drive pos) into a positive pos for comparison
        PositivePoint robotPose = getDrivePoint().toPositivePos();
        Translation2d nodePose = comparisonArray[0].getTranslation();
        //Sets an initial value to a value in the array, so the intial value isn't smaller than all of the values in the array 
        double smallestConeDistance = robotPose.getDistance(nodePose);
        int targetCoordsID = 0;

        //iterates though all points in array
        for (int i = 0; i < comparisonArray.length; i++) {
            //gets the distance between current position and current point in the compariosn array
            nodePose = comparisonArray[i].getTranslation();
            double distance = robotPose.getDistance(nodePose);

            //checks if current distance is smaller than current smallest distance
            if (smallestConeDistance > distance) {
                smallestConeDistance = distance;
                targetCoordsID = i;
            }
        }

        //returns and prints out closets point
        System.out.println(comparisonArray[targetCoordsID].getTranslation().toString());
        return comparisonArray[targetCoordsID];
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // if their speed is larger then the physical max speed, it reduces all speeds
        // until they are smaller than physical max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // sets the modules to desired states
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public void setWheelDirection(Rotation2d angle) {
        frontLeft.setWheelDirection(new SwerveModuleState(0, angle));
        frontRight.setWheelDirection(new SwerveModuleState(0, angle));
        backLeft.setWheelDirection(new SwerveModuleState(0, angle));
        backRight.setWheelDirection(new SwerveModuleState(0, angle));
    }

    public void resetEncoders() {
        // Resets all of the encoders on the robot.
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    @Override
    public void periodic() {
        // this method comes from the subsystem class we inherited. Runs constantly
        // while robot is on

        // changes heading and module states into an x,y coordinate.
        // Updates everything based on the new information it gathers.
        odometer.update(getRotation2d(), getModulePositions());

        headingShuffleBoard.setDouble(getHeading());
        odometerShuffleBoard.setString(getPose().getTranslation().toString());
        pitchSB.setDouble(getPitch());
        rollSB.setDouble(getRoll());
    }
}