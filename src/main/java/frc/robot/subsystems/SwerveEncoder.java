package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;

public class SwerveEncoder {
    //Abslute encoder for swerve drive module
    private CANCoder abosluteEncoder;
    private double offset, reversed;

    public SwerveEncoder(int ID, double offset, boolean reversed) {
        abosluteEncoder = new CANCoder(ID);
        this.offset = offset;
        this.reversed = (reversed ? -1.0 : 1.0);
    }

    public double getPosition() {
        //converts from 0-360 to -PI to PI then applies abosluteEncoder off set
        double angle = abosluteEncoder.getAbsolutePosition()/180*Math.PI;
        angle-=offset;
        angle*=reversed;

        //atan2 funtion range in -PI to PI, so it automaticaly converts (needs the sin and cos to) any input angle to that range
        return Math.atan2(Math.sin(angle), Math.cos(angle));
        
    }
}
