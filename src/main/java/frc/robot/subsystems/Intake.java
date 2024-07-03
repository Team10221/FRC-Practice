package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
	private CANSparkMax angleMotor, intakeMotor, feederMotor;
	private IntakeState currentState = IntakeState.IDLE;

	public enum IntakeState {
		IDLE(0.0, 0.0, IntakeConstants.ANGLE_UP_POSITION),
		INTAKE(IntakeConstants.INTAKE_SPEED, IntakeConstants.FEED_SPEED, IntakeConstants.ANGLE_DOWN_POSITION),
		OUTTAKE(IntakeConstants.OUTTAKE_SPEED, -IntakeConstants.FEED_SPEED, IntakeConstants.ANGLE_DOWN_POSITION);

		public final double intakeSpeed, feederSpeed, anglePosition;

		IntakeState(double intakeSpeed, double feederSpeed, double anglePosition) {
			this.intakeSpeed = intakeSpeed;
			this.feederSpeed = feederSpeed;
			this.anglePosition = anglePosition;
		}
	}

	public Intake() {
		try {
			angleMotor = new CANSparkMax(IntakeConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);
			intakeMotor = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
			feederMotor = new CANSparkMax(IntakeConstants.FEEDER_MOTOR_ID, MotorType.kBrushless);
			configureMotors();
		} catch (Exception e) {
			System.err.println("Error initializing motors: " + e.getMessage());
		}
	}

	private void configureMotors() {
		angleMotor.getPIDController().setP(IntakeConstants.ANGLE_MOTOR_P);
		angleMotor.getPIDController().setI(IntakeConstants.ANGLE_MOTOR_I);
		angleMotor.getPIDController().setD(IntakeConstants.ANGLE_MOTOR_D);
	}

	@Override
	public void periodic() {
		intakeMotor.set(currentState.intakeSpeed);
		feederMotor.set(currentState.feederSpeed);
		angleMotor.getPIDController().setReference(currentState.anglePosition, ControlType.kPosition);
	}

	public void setState(IntakeState state) {
		this.currentState = state;
	}

	public void stopMotors() {
		setState(IntakeState.IDLE);
		intakeMotor.stopMotor();
		feederMotor.stopMotor();
		angleMotor.stopMotor();
	}
}