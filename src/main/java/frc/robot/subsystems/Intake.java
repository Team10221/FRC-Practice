package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeState;
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
        motors.get("intake").set(getState(IntakeState.class).intakeSpeed);
        motors.get("feeder").set(getState(IntakeState.class).feederSpeed);
        setPIDReference("angle", getState(IntakeState.class).anglePosition, ControlType.kPosition);
    }
}