package frc.robot.subsystems.swerveExtras;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.SensorConstants;

public class PosPose2d extends Pose2d {
    public PosPose2d() {
        super();
    }

    public PosPose2d(double x, double y, Rotation2d angle) {
        super(x, y, angle);
    }

    public PosPose2d(PositivePoint posPoint, Rotation2d angle) {
        super(posPoint, angle);
    }

    public PositivePoint getPositivePoint() {
        return new PositivePoint(getX(), getY());
    }

    public FieldPose2d toFieldPose2d(boolean isOnRed) {
        if (isOnRed) {
            return new FieldPose2d(SensorConstants.sizeOfFieldMeters - getX(), getY(), getRotation());
        }
        return new FieldPose2d(getX(), getY(), getRotation());
    }

    public DrivePose2d toDrivePose2d(boolean isOnRed) {
        if (isOnRed) {
            return new DrivePose2d(getX(), getY(), getRotation());
        }
        return new DrivePose2d(getX(), -Math.abs(getY()), getRotation());
    }
}
