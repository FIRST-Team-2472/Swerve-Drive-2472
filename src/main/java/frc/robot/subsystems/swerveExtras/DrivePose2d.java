package frc.robot.subsystems.swerveExtras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SensorConstants;

public class DrivePose2d extends Pose2d{
    public DrivePose2d() {
        super();
    }

    public DrivePose2d(double x, double y, Rotation2d angle) {
        super(x, y, angle);
    }

    public DrivePose2d(PositivePoint posPoint, Rotation2d angle) {
        super(posPoint, angle);
    }

    public DrivePoint getDrivePoint() {
        return new DrivePoint(getX(), getY());
    }

    public FieldPose2d toFieldPose2d(boolean isOnRed) {
        if (isOnRed) {
            return new FieldPose2d(SensorConstants.sizeOfFieldMeters - getX(), getY(), getRotation());
        }
        return new FieldPose2d(getX(), Math.abs(getY()), getRotation());
    }

    public PosPose2d toPosPose2d(boolean isOnRed) {
        if (isOnRed) {
            return new PosPose2d(getX(), getY(), getRotation());
        }
        return new PosPose2d(getX(), Math.abs(getY()), getRotation());
    }
}
