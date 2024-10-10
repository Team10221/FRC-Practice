package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.ShooterConstants.AngleState;
import frc.robot.Constants.IntakeConstants.AngleMotorPID;

public class Intake extends Subsystem {
    private Double val = 0.5;

    public Intake() {
        super(IntakeState.class, AngleState.class);
        addMotor("intakeMotor", new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless));
        addMotor("intakeAngleMotor", new CANSparkMax(IntakeConstants.ANGLE_MOTOR_ID, MotorType.kBrushless));
        addMotor("intakeFeederMotor", new CANSparkMax(IntakeConstants.FEEDER_MOTOR_ID, MotorType.kBrushless));
        addPIDValues("intakeAngle", AngleMotorPID.class);
    }

    @Override
    protected void updateMotors() {
        // example use of static states - the values here are from the original enum
        motors.get("intakeMotor").set(getState(IntakeState.class).intakeSpeed);
        motors.get("intakeFeederMotor").set(getState(IntakeState.class).feederSpeed);

        // example use of dynamic states
        setPIDReference("intakeAngleMotor", getStateValue(IntakeState.class, "anglePosition"), Control.POSITION);
    }

    // example function showing dynamic state control
    int value;

    public void doSomething() {
        // set the angle state to up
        setState(AngleState.UP);

        // change the current state position to 0.5
        modifyStateValue(AngleState.class, "position", 0.5);

        // if there's only one enum field, we don't need the key
        modifyStateValue(AngleState.class, 0.5);

        // get the current state position
        value = getStateValue(AngleState.class, "position");

        // similarly, if there's only one field, we don't need the key
        value = getStateValue(AngleState.class);

        // set the angle state to resting
        setState(AngleState.RESTING);

        // there's more we can do, it's also possible to create a hook
        // through setting a hook, the state will "point" back to an object
        // we define, meaning if we modify the object, it'll reflect in the state
        setHook(AngleState.UP, val);

        removeHook(AngleState.UP, 0);

        // now, if we modify val, it'll modify the state for when the
        // intake is pointed upwards
        val = 0.6;
        // luckily, java autoboxing significantly simplifies implementation
        // even though val is a Double object, we can set it to a primative
        // on compilation, it's "boxed" TODO: finish this explanation
    }
}
