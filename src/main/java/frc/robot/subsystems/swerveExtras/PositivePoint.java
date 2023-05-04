package frc.robot.subsystems.swerveExtras;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.SensorConstants;

public class PositivePoint extends Translation2d {

    public PositivePoint() {
        super();
    }

    //These values are expected to be positive
    public PositivePoint(double m_x, double m_y) {
        super(m_x, m_y);
    }

    public PositivePoint(Translation2d point) {
        super(point.getX(), point.getY());
    }

    public FieldPoint toFieldPos(boolean isOnRed) {
        if (isOnRed) {
            return new FieldPoint(SensorConstants.sizeOfFieldMeters - getX(), getY());
        }
        return new FieldPoint(getX(), getY());
    }

    public DrivePoint toDrivePos(boolean isOnRed) {
        if (isOnRed) {
            return new DrivePoint(getX(), getY());
        }
        return new DrivePoint(getX(), -Math.abs(getY()));
    }
}