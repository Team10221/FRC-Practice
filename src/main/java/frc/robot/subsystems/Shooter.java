package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.AngleState;
import frc.robot.Constants.ShooterConstants.ShooterState;
import frc.robot.util.Subsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends Subsystem {
    public Shooter() {
        super(AngleState.class, ShooterState.class);
        addMotor("top", new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_UP_ID, MotorType.kBrushless));
        addMotor("bottom", new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_DOWN_ID, MotorType.kBrushless));
        addMotor("angle", new CANSparkMax(ShooterConstants.ANGLE_MOTOR_ID, MotorType.kBrushless));
        setName("Shooter");
        configureMotors();
    }

    private void configureMotors() {
        motors.get("bottom").setInverted(true);
        addPIDController("angle", ShooterConstants.ANGLE_PID_P, ShooterConstants.ANGLE_PID_I, ShooterConstants.ANGLE_PID_D);
    }

    @Override
    protected void updateMotors() {
        motors.get("top").set(getState(ShooterState.class).topSpeed);
        motors.get("bottom").set(getState(ShooterState.class).bottomSpeed);
        pidControllers.get("angle").setSetpoint(getState(AngleState.class).position);
        motors.get("angle").set(pidControllers.get("angle").calculate(((CANSparkMax)motors.get("angle")).getEncoder().getPosition()));
    }
}