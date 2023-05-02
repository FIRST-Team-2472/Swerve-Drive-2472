package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, rightTriggerFunction,
            leftTriggerFunction;
    private final Supplier<Integer> xboxPov;
    private final Supplier<Boolean> fieldOrientedFunction;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Double> rightTriggerFunction,
            Supplier<Double> leftTriggerFunction, Supplier<Integer> xboxPov) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.rightTriggerFunction = rightTriggerFunction;
        this.leftTriggerFunction = leftTriggerFunction;
        this.xboxPov = xboxPov;

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

        boolean smallRotateCounterclockwiseFunction = leftTriggerFunction.get() >= .6 ? true : false;
        boolean smallRotateClockwiseFunction = rightTriggerFunction.get() >= .6 ? true : false;

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? Math.copySign(Math.pow(xSpeed * slow, 2), xSpeed) : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? Math.copySign(Math.pow(ySpeed * slow, 2), ySpeed) : 0.0;

        if (!(smallRotateClockwiseFunction || smallRotateCounterclockwiseFunction))
            turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband
                    ? Math.copySign(Math.pow(turningSpeed * slow, 2), turningSpeed) : 0.0;
        else if (smallRotateClockwiseFunction)
            turningSpeed = .25;
        //technicly this if is unecissary, but it makes it more readable
        else if (smallRotateCounterclockwiseFunction)
            turningSpeed = -.25;

        if (xboxPov.get() == 90)
            ySpeed = .2;
        else if (xboxPov.get() == 270)
            ySpeed = -.2;
        else if (xboxPov.get() == 0)
            xSpeed = .2;
        else if (xboxPov.get() == 180)
            xSpeed = -.2;

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
