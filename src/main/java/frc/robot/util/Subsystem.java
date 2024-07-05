package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstract base class for robot subsystems.
 */
public abstract class Subsystem extends SubsystemBase {
    private final Map<Class<? extends Enum<?>>, Mutable<? extends Comparable<?>>> values;
    private final Map<Class<? extends Enum<?>>, Enum<?>> states;
    public final Map<String, MotorController> motors;

    /**
     * Constructs a Subsystem with initial states and values for the given enum classes.
     * @param enumClasses The enum classes to initialize states and values from.
     */
    @SafeVarargs
    public Subsystem(Class<? extends Enum<?>>... enumClasses) {
        this.values = new HashMap<>();
        this.states = new HashMap<>();
        this.motors = new HashMap<>();

        // Convert each enum to a Mutable and extract enum constants
        for (Class <? extends Enum<?>> clazz : enumClasses) {
            values.put(clazz, EnumToMutable.translate(clazz));
            states.put(clazz, clazz.getEnumConstants()[0]);
        }
    }

    /**
     * Gets the current state for a given enum class.
     * @param <E> The enum type.
     * @param stateClass The class of the enum.
     * @return The current state, or the first enum constant if no state is set.
     */
    public <E extends Enum<E>> E getState(Class<E> stateClass) {
        // Get the state from the enum class
        Enum<?> state = states.get(stateClass);
        
        // Check if the state is null
        if (state == null) {
            System.err.println("WARNING: No state found for class: " + stateClass.getSimpleName());
            
            E[] enumConstants = stateClass.getEnumConstants();
            if (enumConstants.length > 0) { 
                // Return the first constant if constants are present
                return enumConstants[0];
            } else { 
                // Return null if no enum constants are found
                System.err.println("ERROR: No enum constants found for class: " + stateClass.getSimpleName());
                return null;
            }
        }

        // Return the state
        return stateClass.cast(state);
    }

    /**
     * Sets the state for a given enum class.
     * @param <E> The enum type.
     * @param state The state to set.
     */
    public <E extends Enum<E>> void setState(E state) {
        // Check if the state is null
        if (state == null) {
            System.err.println("ERROR: Attempted to set null state for subystem " + getName());
            return;
        }

        // Update the HashMap
        states.put(state.getDeclaringClass(), state);
    }

    /**
     * Retrieves a specified value from a mutable state instance.
     * @param <T> The enum type.
     * @param enumClass The original enum class.
     * @param instanceName The name of the state to retrieve.
     * @param key The field name to retrieve.
     * @return The value of the specified field in the state.
     */
    public <T extends Enum<T>> Object getStateValue(Class<?> enumClass, String instanceName, String key) {
        return values.get(enumClass).getInstance(instanceName).get(key);
    }

    /**
     * Modifies a value in a Mutable state instance.
     * @param <T> The type of the value being set.
     * @param enumClass The original enum class.
     * @param instanceName The name of the state to modify.
     * @param key The field name to update.
     * @param value The new value to set.
     */
    @SuppressWarnings("unchecked")
    public <T extends Comparable<T>> void modifyStateValue(Class<?> enumClass, String instanceName, String key, T value) {
        Mutable<T> mutable = (Mutable<T>) values.get(enumClass);
        mutable.getInstance(instanceName).set(key, value);
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
     * Adds PID values to a motor's PID controller
     * @param name The motor's name
     * @param constants The PID constants
     */
    protected void addPIDValues(String name, Class<?> constants) {
        MotorController motor = motors.get(name);

        // Check if the motor name is null
        if (name == null) {
            System.err.println("ERROR: Attempted to add PID values with null name to subsystem " + getName());
            return;
        }

        // Get the PID values from the class's fields
        double kP = getClassFields(constants, "kP", "P");
        double kI = getClassFields(constants, "kI", "I");
        double kD = getClassFields(constants, "kD", "D");

        // Use the integrated spark max PID controller if the motor is a spark max
        if (motor instanceof CANSparkMax) {
            SparkPIDController pidController = ((CANSparkMax) motor).getPIDController();
            pidController.setP(kP);
            pidController.setI(kI);
            pidController.setD(kD);
        }
    }

    /**
     * Sets a PID setpoint with a specified control type
     * @param name The motor's name
     * @param setpoint The PID setpoint
     * @param controlType The PID controller mode
     */
    protected void setPIDReference(String name, double setpoint, ControlType controlType) {
        MotorController motor = motors.get(name);

        // Use SparkPIDController's built in PID control if the motor controller is a CAN spark max
        if (motor instanceof CANSparkMax) {
            ((CANSparkMax) motor).getPIDController().setReference(setpoint, controlType);
        }
    }

    /**
     * Helper function to get a double field from a class to be used by getClassFields
     * @param clazz The static class
     * @param fieldName The static field name
     * @return A Double object representing the double field if present, null otherwise
     */
    private Double getClassField(Class<?> clazz, String fieldName) {
        try {
            return clazz.getField(fieldName).getDouble(null);
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Helper function to get a double field from a possibility of multiple fields
     * @param clazz The static class
     * @param fieldNames The possible field names as arguments
     * @return A double value for the first valid field supplied as an argument
     */
    private double getClassFields(Class<?> clazz, String... fieldNames) {
        for (String fieldName : fieldNames) {
            Double field = getClassField(clazz, fieldName);
            if (field != null) {
                return field;
            }
        }
        return 0.0;
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
        states.forEach((stateClass, state) -> 
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