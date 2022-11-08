package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;

public class SwerveEncoder {
    //Abslute encoder for swerve drive module
    private CANCoder abosluteEncoder;

    public SwerveEncoder(int ID) {
        abosluteEncoder = new CANCoder(ID);
    }

    public double getPosition() {
        return abosluteEncoder.getAbsolutePosition()/180*Math.PI-Math.PI;
    }
}
