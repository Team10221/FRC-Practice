package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.AngleState;
import frc.robot.Constants.ShooterConstants.ShooterState;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
    private CANSparkMax topShooterMotor, bottomShooterMotor, angleMotor;
    private AngleState angleState = AngleState.RESTING;
    private ShooterState shooterState = ShooterState.IDLE;

    public Shooter() {
        try {
            topShooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_UP_ID, MotorType.kBrushless);
            bottomShooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_DOWN_ID, MotorType.kBrushless);
            angleMotor = new CANSparkMax(ShooterConstants.ANGLE_MOTOR_ID, MotorType.kBrushless);
            configureMotors();
        } catch (Exception e) {
            System.err.println("Error initializing motors: " + e.getMessage());
        }
    }

    private void configureMotors() {
        bottomShooterMotor.setInverted(true);
        angleMotor.getPIDController().setP(ShooterConstants.ANGLE_PID_P);
        angleMotor.getPIDController().setI(ShooterConstants.ANGLE_PID_I);
        angleMotor.getPIDController().setD(ShooterConstants.ANGLE_PID_D);
    }

    @Override
    public void periodic() {
        topShooterMotor.set(shooterState.topSpeed);
        bottomShooterMotor.set(shooterState.bottomSpeed);
        angleMotor.getPIDController().setReference(angleState.position, ControlType.kPosition);

        SmartDashboard.putString("Shooter State", shooterState.toString());
        SmartDashboard.putString("Shooter Angle State", angleState.toString());
        SmartDashboard.putNumber("Shooter Angle Position", angleMotor.getEncoder().getPosition());
    }

    public void setShooterState(ShooterState state) {
        this.shooterState = state;
    }

    public void setAngleState(AngleState state) {
        this.angleState = state;
    }

    public AngleState getAngleState() {
        return this.angleState;
    }

    public ShooterState getShooterState() {
        return this.shooterState;
    }

    public boolean isAtTargetAngle() {
        return Math.abs(angleMotor.getEncoder().getPosition() - angleState.position) < ShooterConstants.ACCURACY_THRESHOLD;
    }

    public void stop() {
        shooterState = ShooterState.IDLE;
        angleState = AngleState.RESTING;
        topShooterMotor.stopMotor();
        bottomShooterMotor.stopMotor();
        angleMotor.stopMotor();
    }
}