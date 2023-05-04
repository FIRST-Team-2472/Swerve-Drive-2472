package frc.robot.subsystems.swerveExtras;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SensorConstants;

public class DrivePoint extends Translation2d {
    public DrivePoint() {
        super();
    }

    public DrivePoint(double m_x, double m_y) {
        super(m_x, m_y);
    }

    public DrivePoint(Translation2d point) {
        super(point.getX(), point.getY());
    }

    public FieldPoint toFieldPos(boolean isOnRed) {
        if (isOnRed) {
            return new FieldPoint(SensorConstants.sizeOfFieldMeters - getX(), getY());
        }
        return new FieldPoint(getX(), Math.abs(getY()));
    }

    public PositivePoint toPositivePos(boolean isOnRed) {
        if (isOnRed) {
            return new PositivePoint(getX(), getY());
        }
        return new PositivePoint(getX(), Math.abs(getY()));
    }
}
