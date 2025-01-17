package frc.robot.subsystems;

import frc.lib.motor.Motor;
import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.ShooterConstants.AngleState;

public class Intake extends Subsystem<Double> {
    private Motor intakeMotor = Motor.neo(IntakeConstants.INTAKE_MOTOR_ID).setPID(IntakeConstants.PID);
    private Motor intakeAngleMotor = Motor.neo(IntakeConstants.ANGLE_MOTOR_ID);
    private Motor intakeFeederMotor = Motor.neo(IntakeConstants.FEEDER_MOTOR_ID);

    public Intake() {
        super(IntakeState.class, AngleState.class);
    }

    protected void updateMotors() {
        intakeMotor.set(getState(IntakeState.class).intakeSpeed);
        intakeFeederMotor.set(getState(IntakeState.class).feederSpeed);
        intakeAngleMotor.setReference(getState(AngleState.class).position);
    }
}
