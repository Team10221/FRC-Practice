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

/*
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
*/