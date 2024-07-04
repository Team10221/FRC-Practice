package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstract base class for robot subsystems
 */
public abstract class Subsystem extends SubsystemBase {
    private final Map<Class<? extends Enum<?>>, Enum<?>> currentStates;
    public final Map<String, MotorController> motors = new HashMap<>();
    public final Map<String, PIDController> pidControllers = new HashMap<>();

    /**
     * Constructs a Subsystem with initial states for the given enum classes.
     * @param enumClasses The enum classes to initialize states for
     */
    @SafeVarargs
    public Subsystem(Class<? extends Enum<?>>... enumClasses) {
        this.currentStates = new HashMap<>();
        for (Class <? extends Enum<?>> clazz : enumClasses) {
            currentStates.put(clazz, clazz.getEnumConstants()[0]);
        }
    }

    /**
     * Sets the state for a given enum type
     * @param <E> The enum type
     * @param state The state to set
     */
    public <E extends Enum<E>> void setState(E state) {
        // Check if a null state is attempting to be set
        if (state == null) {
            System.err.println("ERROR: Attempted to set null state for subystem " + getName());
            return;
        }

        currentStates.put(state.getDeclaringClass(), state);
    }

    /**
     * Gets the current state for a given enum class
     * @param <E> The enum type
     * @param stateClass The class of the enum
     * @return The current state, or the first enum constant if no state is set
     */
    public <E extends Enum<E>> E getState(Class<E> stateClass) {
        Enum<?> state = currentStates.get(stateClass);

        // TBD: Add good comment
        if (state == null) {
            // Log the error
            System.err.println("WARNING: No state found for class: " + stateClass.getSimpleName());
            SmartDashboard.putBoolean(getName() + " State Error", true);

            // Return the first enum constant as a default state
            E[] enumConstants = stateClass.getEnumConstants();
            if (enumConstants.length > 0) {
                return enumConstants[0];
            } else {
                // Return null if no enum constants are present
                System.err.println("ERROR: No enum constants found for class: " + stateClass.getSimpleName());
                return null;
            }
        }

        return stateClass.cast(state);
    }

    /**
     * Adds a motor to the subsystem
     * @param name The name of the motor
     * @param motor The motor controller
     */
    protected void addMotor(String name, MotorController motor) {
        // Check if the provided name or motor are null
        if (name == null || motor == null) {
            System.err.println("ERROR: Attempted to add null motor or name to " + getName());
            return;
        }

        // Check if the motors map already contains the name as a key
        if (motors.containsKey(name)) {
            System.err.println("WARNING: Overwriting existing motor '" + name + "' in " + getName());
        }

        motors.put(name, motor);
    }

    /**
     * Adds a PID controller to the subsystem
     * @param name The name of the PID controller
     * @param kP The proportional gain
     * @param kI The integral gain
     * @param kD The derivative gain
     */
    protected void addPIDController(String name, double kP, double kI, double kD) {
        // Check if the provided name is null
        if (name == null) {
            System.err.println("ERROR: Attempted to add PID controller with null name to " + getName());
            return;
        }

        // Check if the pid controllers map already contains the name as a key
        if (pidControllers.containsKey(name)) {
            System.err.println("WARNING: Overwriting existing PID controller '" + name + "' in " + getName());
        }

        pidControllers.put(name, new PIDController(kP, kI, kD));
    }

    /**
     * Checks if a motor is at the target position
     * @param motorName The name of the motor
     * @param targetPosition The target position
     * @param threshold The acceptable threshold for considering the position reached
     * @return true if the motor at the target position, false otherwise
     */
    public boolean isAtTargetPosition(String motorName, double targetPosition, double threshold) {
        MotorController motor = motors.get(motorName);

        // Check if the motor doesn't exist
        if (motor == null) {
            System.err.println("ERROR: No motor found with name '" + motorName + "' in " + getName());
            return false;
        }

        // Check if the motor controller is a spark max
        if (motor instanceof CANSparkMax) {
            return Math.abs(((CANSparkMax) motor).getEncoder().getPosition() - targetPosition) < threshold;
        }

        // Return false if the motor controller isn't supported
        return false;
    }

    /**
     * Abstract method to update motor outputs. Must be implemented by subclasses.
     */
    protected abstract void updateMotors();

    /**
     * Updates the SmartDashboard with current states and motor outputs.
     */
    protected void updateSmartDashboard() {
        currentStates.forEach((stateClass, state) -> 
            SmartDashboard.putString(getName() + " " + stateClass.getSimpleName(), state.toString()));
        motors.forEach((name, motor) -> 
            SmartDashboard.putNumber(getName() + " " + name + " Output", motor.get()));
    }

    /**
     * Periodic method called repeatedly. Updates motors and SmartDashboard.
     */
    @Override
    public void periodic() {
        updateMotors();
        updateSmartDashboard();
    }

    /**
     * Stops all motors in a subsystem.
     */
    public void stopMotors() {
        motors.values().forEach(MotorController::stopMotor);
    }
}