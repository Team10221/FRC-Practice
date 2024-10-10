package frc.robot.subsystems;

import frc.lib.subsystem.Subsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.AngleState;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.Constants.ShooterConstants.AngleMotorPID;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends Subsystem {
    private Double shooterAngle = getState(AngleState.class).position;

    public Shooter() {
        // the constructor is supplied with 2 enum classes, one per motor
        super(AngleState.class, ShooterState.class);

        // creates 3 new motors using the addMotor function
        // added to the motors hashmap, the key being the motor name and the value being
        // the motor itself
        addMotor("shooterTopMotor", new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_UP_ID, MotorType.kBrushless));
        addMotor("shooterBottomMotor", new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_DOWN_ID, MotorType.kBrushless));
        addMotor("shooterAngleMotor", new CANSparkMax(ShooterConstants.ANGLE_MOTOR_ID, MotorType.kBrushless));

        // configures motors after decleration
        // includes setting PIDs, inverting, etc...
        motors.get("shooterBottomMotor").setInverted(true);
        addPIDValues("shooterAngleMotor", AngleMotorPID.class);
    }

    // for each motor
    @Override
    protected void updateMotors() {
        motors.get("shooterTopMotor").set(getState(ShooterState.class).topSpeed);
        motors.get("shooterBottomMotor").set(getState(ShooterState.class).bottomSpeed);
        setPIDReference("shooterAngleMotor", getState(AngleState.class).position, Control.POSITION);
    }
}
