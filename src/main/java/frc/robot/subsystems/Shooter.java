package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.AngleState;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.Constants.ShooterConstants.AngleMotorPID;
import frc.robot.util.Subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends Subsystem {
    public Shooter() {
        // the constructor is supplied with 2 enum classes, one per motor
        super(AngleState.class, ShooterState.class);

        // creates 3 new motors using the addMotor function
        // added to the motors hashmap, the key being the motor name and the value being the motor itself
        addMotor("top", new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_UP_ID, MotorType.kBrushless));
        addMotor("bottom", new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_DOWN_ID, MotorType.kBrushless));
        addMotor("angle", new CANSparkMax(ShooterConstants.ANGLE_MOTOR_ID, MotorType.kBrushless));

        // sets the name of the subsystem
        setName("Shooter");

        // configures motors after decleration
        // includes setting PIDs, inverting, etc...
        motors.get("bottom").setInverted(true);
        addPIDValues("angle", AngleMotorPID.class);
    }

    // for each motor
    @Override
    protected void updateMotors() {
        motors.get("top").set(getState(ShooterState.class).topSpeed);
        motors.get("bottom").set(getState(ShooterState.class).bottomSpeed);
        setPIDReference("angle", getState(AngleState.class).position, Control.POSITION);
    }
}