package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.Constants.DriveConstants;

public class SwerveEncoder {
    //Abslute encoder for swerve drive module
    private CANCoder abosluteEncoder;
    private double offset;
    private int reversed;

    public SwerveEncoder(int ID, double offset, boolean reversed) {
        abosluteEncoder = new CANCoder(ID);
        this.offset = offset;
        this.reversed = (reversed ? -1.0 : 1.0);
    }

    public double getPosition() {
        //converts from 0-360 to -PI to PI then applies abosluteEncoder off set
        double angle = abosluteEncoder.getAbsolutePosition()/180*Math.PI-Math.PI;
        angle = angle * reversed;
        
        //though fancy math wizardery this applies the offset then corrects it's domain to [-pi,pi] (this is overkill)
        //this function could convert any angle to an angle of [-pi, pi]
        return 2*Math.asin(sin(angle-offset));

    }
}
