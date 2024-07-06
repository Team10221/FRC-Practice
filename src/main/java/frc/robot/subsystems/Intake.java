package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Constants.ShooterConstants.AngleState;
import frc.robot.Constants.IntakeConstants.AngleMotorPID;
import frc.robot.util.Subsystem;

public class Intake extends Subsystem {
    public Intake() {
        super(IntakeState.class);
        addMotor("angle", new CANSparkMax(IntakeConstants.ANGLE_MOTOR_ID, MotorType.kBrushless));
        addMotor("intake", new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless));
        addMotor("feeder", new CANSparkMax(IntakeConstants.FEEDER_MOTOR_ID, MotorType.kBrushless));
        addPIDValues("angle", AngleMotorPID.class);
        setName("Intake");
    }

    @Override
    protected void updateMotors() {
        // example use of static states - the values here are from the original enum
        motors.get("intake").set(getState(IntakeState.class).intakeSpeed);
        motors.get("feeder").set(getState(IntakeState.class).feederSpeed);

        // example use of dynamic states
        setPIDReference("angle", getStateValue(IntakeState.class, "anglePosition"), Control.POSITION);
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
    }
}