package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class Subsystem extends SubsystemBase {
    private final Map<Class<? extends Enum<?>>, Enum<?>> currentStates;
    public final Map<String, MotorController> motors = new HashMap<>();
    public final Map<String, PIDController> pidControllers = new HashMap<>();

    @SafeVarargs
    public Subsystem(Class<? extends Enum<?>>... enumClasses) {
        this.currentStates = new HashMap<>();
        for (Class <? extends Enum<?>> clazz : enumClasses) {
            currentStates.put(clazz, clazz.getEnumConstants()[0]);
        }
    }

    public <E extends Enum<E>> void setState(E state) {
        currentStates.put(state.getDeclaringClass(), state);
    }

    public <E extends Enum<E>> E getState(Class<E> stateClass) {
        return stateClass.cast(currentStates.get(stateClass));
    }

    protected void addMotor(String name, MotorController motor) {
        motors.put(name, motor);
    }

    protected void addPIDController(String name, double kP, double kI, double kD) {
        pidControllers.put(name, new PIDController(kP, kI, kD));
    }

    @Override
    public void periodic() {
        updateMotors();
        updateSmartDashboard();
    }

    protected abstract void updateMotors();

    protected void updateSmartDashboard() {
        currentStates.forEach((stateClass, state) -> 
            SmartDashboard.putString(getName() + " " + stateClass.getSimpleName(), state.toString()));
        motors.forEach((name, motor) -> 
            SmartDashboard.putNumber(getName() + " " + name + " Output", motor.get()));
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