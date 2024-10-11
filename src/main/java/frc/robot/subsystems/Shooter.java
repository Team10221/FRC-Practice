package frc.robot.subsystems;

import frc.lib.subsystem.Subsystem;
import frc.lib.subsystem.util.Motor;
import frc.lib.subsystem.util.PID;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.AngleState;
import frc.robot.Constants.ShooterConstants.ShooterState;

public class Shooter extends Subsystem {
    private Motor shooterTopMotor = Motor.neo(ShooterConstants.SHOOTER_MOTOR_UP_ID);
    private Motor shooterBottomMotor = Motor.neo(ShooterConstants.SHOOTER_MOTOR_DOWN_ID).invert();
    private Motor shooterAngleMotor = Motor.neo(ShooterConstants.ANGLE_MOTOR_ID).setPID(new PID(0, 0, 0));

    public Shooter() {
        super(AngleState.class, ShooterState.class);
    }

    protected void updateMotors() {
        shooterTopMotor.set(getState(ShooterState.class).topSpeed);
        shooterBottomMotor.set(getState(ShooterState.class).bottomSpeed);
        shooterAngleMotor.setReference(getState(AngleState.class).position);
    }
}
