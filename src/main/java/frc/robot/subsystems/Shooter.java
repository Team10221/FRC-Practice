package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
    private CANSparkMax shooterMotorUp, shooterMotorDown, angleMotor;

    public Shooter() {
        shooterMotorUp = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_UP_ID, MotorType.kBrushless);
        shooterMotorDown = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_DOWN_ID, MotorType.kBrushless);
        shooterMotorDown.setInverted(true);

        angleMotor = new CANSparkMax(ShooterConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);

        angleMotor.getPIDController().setP(ShooterConstants.ANGLE_PID_P);
        angleMotor.getPIDController().setI(ShooterConstants.ANGLE_PID_I);
        angleMotor.getPIDController().setD(ShooterConstants.ANGLE_PID_D);
    }

    public void setShooterSpeed(double upSpeed, double downSpeed) {
        shooterMotorUp.set(upSpeed);
        shooterMotorDown.set(downSpeed);
    }

    public void setShooterState(ShooterConstants.ShooterState state) {
        switch (state) {
            case ON:
                setShooterSpeed(ShooterConstants.SHOOTER_SPEED_UP, ShooterConstants.SHOOTER_SPEED_DOWN);
                break;
            case OFF:
                setShooterSpeed(0.0, 0.0);
        }
    }

    public void setAngle(ShooterConstants.AngleState angleState) {
        angleMotor.getPIDController().setReference(angleState.position, ControlType.kPosition);
    }

    public void stop() {
        shooterMotorUp.stopMotor();
        shooterMotorDown.stopMotor();
        angleMotor.stopMotor();
    }

}