package frc.robot.subsystems.swerveExtras;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SensorConstants;

public class FieldPoint extends Translation2d {
    public FieldPoint() {
        super();
    }

    public FieldPoint(double m_x, double m_y) {
        super(m_x, m_y);
    }

    public FieldPoint(Translation2d point) {
        super(point.getX(), point.getY());
    }

    public DrivePoint toDrivePos(boolean isOnRed) {
        if (isOnRed) {
            return new DrivePoint(SensorConstants.sizeOfFieldMeters - getX(), getY());
        }
        return new DrivePoint(getX(), -Math.abs(getY()));
    }

    public PositivePoint toPositivePos(boolean isOnRed) {
        if (isOnRed) {
            return new PositivePoint(SensorConstants.sizeOfFieldMeters - getX(), getY());
        }
        return new PositivePoint(getX(), getY());
    }
}