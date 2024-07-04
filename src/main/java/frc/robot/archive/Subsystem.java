package frc.robot.archive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// An older implementation of util/Subsystem.java, not to be used

public abstract class Subsystem<T extends Enum<T>> extends SubsystemBase {
    public final T[] states;
    public T currentState;
    public final Map<String, MotorController> motors = new HashMap<>();
    public final Map<String, PIDController> pidControllers = new HashMap<>();

    public Subsystem(Class<T> enumClass) {
        this.states = enumClass.getEnumConstants();
        this.currentState = states[0];
    }

    protected void addMotor(String name, MotorController motor) {
        motors.put(name, motor);
    }

    protected void addPIDController(String name, double kP, double kI, double kD) {
        pidControllers.put(name, new PIDController(kP, kI, kD));
    }

    public void setState(T state) {
        this.currentState = state;
    }

    public T getState() {
        return this.currentState;
    }

    @Override
    public void periodic() {
        updateMotors();
        updateSmartDashboard();
    }

    protected abstract void updateMotors();

    protected void updateSmartDashboard() {
        SmartDashboard.putString(getName() + " State", currentState.toString());
        motors.forEach((name, motor) -> 
            SmartDashboard.putNumber(getName() + " " + " Output", motor.get()));
    }

    public void stopMotors() {
        motors.values().forEach(MotorController::stopMotor);
    }

    public boolean isAtTargetPosition(String motorName, double targetPosition, double threshold) {
        MotorController motor = motors.get(motorName);
        if (motor instanceof CANSparkMax) {
            return Math.abs(((CANSparkMax) motor).getEncoder().getPosition() - targetPosition) < threshold;
        }
        return false;
    }
}
