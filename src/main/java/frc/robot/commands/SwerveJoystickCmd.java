package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.intializeJoystickRunFromField();
        System.out.println("Swerve Joystick contoslled!");
    }

    @Override
    public void execute() {
        double slow = 1;
        if (fieldOrientedFunction.get())
            slow = .5;

        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband & square output for more percise movement at low speed
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? Math.copySign(Math.pow(xSpeed * slow, 2), xSpeed) : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? Math.copySign(Math.pow(ySpeed * slow, 2), ySpeed) : 0.0;

        swerveSubsystem.excuteJoystickRunFromField(xSpeed, ySpeed, turningSpeed);
    }

    @Override
    public void end(boolean interrupted) {

        // swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
