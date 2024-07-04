package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.util.Subsystem;

public class Intake extends Subsystem {
    public Intake() {
        super(IntakeState.class);
        addMotor("angle", new CANSparkMax(IntakeConstants.ANGLE_MOTOR_ID, MotorType.kBrushless));
        addMotor("intake", new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless));
        addMotor("feeder", new CANSparkMax(IntakeConstants.FEEDER_MOTOR_ID, MotorType.kBrushless));
        addPIDController("angle", IntakeConstants.ANGLE_MOTOR_P, IntakeConstants.ANGLE_MOTOR_I, IntakeConstants.ANGLE_MOTOR_D);
    }

    @Override
    protected void updateMotors() {
		IntakeState currentState = getState(IntakeState.class);

		motors.get("intake").set(getState(IntakeState.class).intakeSpeed);
		motors.get("feeder").set(getState(IntakeState.class).feederSpeed);
        
        double targetPosition = (currentState == IntakeState.IDLE) ? 
            Constants.IntakeConstants.ANGLE_UP_POSITION : 
            Constants.IntakeConstants.ANGLE_DOWN_POSITION;
        pidControllers.get("angle").setSetpoint(targetPosition);
        motors.get("angle").set(pidControllers.get("angle").calculate(((CANSparkMax)motors.get("angle")).getEncoder().getPosition()));
    }
}