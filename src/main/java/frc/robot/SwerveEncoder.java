package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants.DriveConstants;

public class SwerveEncoder {
    //Abslute encoder for swerve drive module
    private CANCoder abosluteEncoder;
    private double offset;

    public SwerveEncoder(int ID, double offset) {
        abosluteEncoder = new CANCoder(ID);
        this.offset = offset;
    }

    public double getPosition() {
        //converts from 0-360 to -PI to PI then applies abosluteEncoder off set
        double angle = abosluteEncoder.getAbsolutePosition()/180*Math.PI-Math.PI;
        return angle * (DriveConstants.kDriveAbsoluteEncoderReversed ? -1.0 : 1.0);

    }
}
