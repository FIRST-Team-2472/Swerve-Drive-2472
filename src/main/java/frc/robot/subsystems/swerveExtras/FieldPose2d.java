package frc.robot.subsystems.swerveExtras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SensorConstants;

public class FieldPose2d extends Pose2d {
    public FieldPose2d() {
        super();
    }

    public FieldPose2d(double x, double y, Rotation2d angle) {
        super(x, y, angle);
    }

    public FieldPose2d(PositivePoint posPoint, Rotation2d angle) {
        super(posPoint, angle);
    }

    public FieldPoint getFieldPoint() {
        return new FieldPoint(getX(), getY());
    }

    public DrivePose2d toDrivePose2d(boolean isOnRed) {
        if (isOnRed) {
            return new DrivePose2d(SensorConstants.sizeOfFieldMeters - getX(), getY(), getRotation());
        }
        return new DrivePose2d(getX(), -Math.abs(getY()), getRotation());
    }

    public PosPose2d toPosPose2d(boolean isOnRed) {
        if (isOnRed) {
            return new PosPose2d(SensorConstants.sizeOfFieldMeters - getX(), getY(), getRotation());
        }
        return new PosPose2d(getX(), getY(), getRotation());
    }
}
