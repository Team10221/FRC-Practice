package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import com.revrobotics.CANSparkBase;
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

        for (Class <? extends Enum<?>> clazz : enumClasses) {
            values.put(clazz, translate(clazz));
            states.put(clazz, clazz.getEnumConstants()[0]);
        }
    }

    /**
     * Translates an enum class to a Mutable object.
     * @param <E> The enum type.
     * @param <T> The type of values in the enum (assumed to be the same for all fields).
     * @param enumClass The enum class to translate.
     * @return A Mutable object representing the enum.
     */
    @SuppressWarnings("unchecked")
    public static <E extends Enum<E>, T extends Comparable<T>> Mutable<T> translate(Class<? extends Enum<?>> enumClass) {
        Mutable<T> mutable = new Mutable<>();
        for (Enum<?> enumConstant: enumClass.getEnumConstants()) {
            Mutable.Builder<T> builder = new Mutable.Builder<>(enumConstant.name());
            for (Field field : enumClass.getDeclaredFields()) {
                if (!field.isEnumConstant() && !field.isSynthetic()) {
                    field.setAccessible(true);
                    try {
                        T value = (T) field.get(enumConstant);
                        builder.with(field.getName(), value);
                    } catch (IllegalAccessException e) {
                        System.err.println("Error accessing field: " + field.getName());
                    }
                }
            }
            mutable.addInstance(builder.build());
        }
        return mutable;
    }

    /**
     * Gets the current state for a given enum class.
     * @param <E> The enum type.
     * @param stateClass The class of the enum.
     * @return The current state, or the first enum constant if no state is set.
     */
    public <E extends Enum<E>> E getState(Class<E> stateClass) {
        Enum<?> state = states.get(stateClass);
        if (state == null) {
            System.err.println("WARNING: No state found for class: " + stateClass.getSimpleName());
            E[] enumConstants = stateClass.getEnumConstants();
            if (enumConstants.length > 0) { 
                return enumConstants[0];
            } else { 
                System.err.println("ERROR: No enum constants found for class: " + stateClass.getSimpleName());
                return null;
            }
        }
        return stateClass.cast(state);
    }

    /**
     * Sets the state for a given enum class.
     * @param <E> The enum type.
     * @param state The state to set.
     */
    public <E extends Enum<E>> void setState(E state) {
        if (state == null) {
            System.err.println("ERROR: Attempted to set null state for subystem " + getName());
            return;
        }
        states.put(state.getDeclaringClass(), state);
    }

    /**
     * Retrieves the value from the current state when there's only one field.
     * @param <T> The type of the value to retrieve.
     * @param enumClass The enum class representing the state.
     * @return The value of the single field in the current state, or null if an error occurs.
     */
    @SuppressWarnings("unchecked")
    public <T extends Comparable<T>> T getStateValue(Class<?> enumClass) {
        Enum<?> currentState = states.get(enumClass);
        if (currentState == null) {
            System.err.println("ERROR: No current state found for class: " + enumClass.getSimpleName());
            return null;
        }

        Mutable<T> mutable = (Mutable<T>) values.get(enumClass);
        Mutable.Instance<T> instance = mutable.getInstance(currentState.name());
        if (instance == null) {
            System.err.println("ERROR: No instance found for state: " + currentState.name());
            return null;
        }

        Set<String> fieldNames = instance.getKeys();
        if (fieldNames.size() != 1) {
            System.err.println("ERROR: Expected exactly one field, but found " + fieldNames.size() + " for state: " + currentState.name());
            return null;
        }

        String singleFieldName = fieldNames.iterator().next();
        return instance.get(singleFieldName);
    }

    /**
     * Retrieves a specified value from the current state for an enum.
     * @param <T> The enum type.
     * @param enumClass The original enum class.
     * @param key The field name to retrieve.
     * @return The value of the specified field in the state.
     */
    @SuppressWarnings("unchecked")
    public <T extends Comparable<T>> T getStateValue(Class<?> enumClass, String key) {
        Enum<?> currentState = states.get(enumClass);

        if (currentState == null) {
            System.err.println("ERROR: No current state found for class: " + enumClass.getSimpleName());
            return null;
        }

        Mutable<T> mutable = (Mutable<T>) values.get(enumClass);
        return mutable.getInstance(currentState.name()).get(key);
    }

    /**
     * Retrieves a specified value from a mutable state instance.
     * @param <T> The type of the value to retrieve.
     * @param enumClass The original enum class.
     * @param key The field name to retrieve.
     * @param instanceName Optional. The name of the state to retrieve. If null, uses the current state.
     * @return The value of the specified field in the state.
     */
    @SuppressWarnings("unchecked")
    public <T extends Comparable<T>> T getStateValue(Class<? extends Enum<?>> enumClass, String key, String instanceName) {
        Mutable<T> mutable = (Mutable<T>) values.get(enumClass);
        if (mutable == null) {
            System.err.println("ERROR: No Mutable found for enum class " + enumClass.getSimpleName());
            return null;
        }

        String stateName;
        if (instanceName == null) {
            System.err.println("WARNING: provided instanceName is null, defaulting to current state name");

            Enum<?> currentState = states.get(enumClass);
            if (currentState == null) {
                System.err.println("ERROR: No current state found for class: " + enumClass.getSimpleName());
                return null;
            }
            stateName = currentState.name();
        } else {
            stateName = instanceName;
        }

        Mutable.Instance<T> instance = mutable.getInstance(stateName);
        if (instance == null) {
            System.err.println("ERROR: No instance found with name " + stateName);
            return null;
        }

        return instance.get(key);
    }

    /**
     * Modifies the value of the current state when there's only one field.
     * @param <T> The type of the value to set.
     * @param enumClass The enum class representing the state.
     * @param value The new value to set for the single field.
     */
    @SuppressWarnings("unchecked")
    public <T extends Comparable<T>> void modifyStateValue(Class<? extends Enum<?>> enumClass, T value) {
        Enum<?> currentState = states.get(enumClass);
        if (currentState == null) {
            System.err.println("ERROR: No current state found for class: " + enumClass.getSimpleName());
            return;
        }

        Mutable<T> mutable = (Mutable<T>) values.get(enumClass);
        Mutable.Instance<T> instance = mutable.getInstance(currentState.name());

        if (instance == null) {
            System.err.println("ERROR: No instance found for state: " + currentState.name());
            return;
        }

        Set<String> fieldNames = instance.getKeys();
        if (fieldNames.size() != 1) {
            System.err.println("ERROR: Expected exactly one field, but found " + fieldNames.size() + " for state: " + currentState.name());
            return;
        }

        String singleFieldName = fieldNames.iterator().next();
        instance.set(singleFieldName, value);
    }

    @SuppressWarnings("unchecked")
    public <T extends Comparable<T>> void modifyStateValue(Class<? extends Enum<?>> enumClass, String key, T value) {
        Enum<?> currentState = states.get(enumClass);

        if (currentState == null) {
            System.err.println("ERROR: No current state found for class: " + enumClass.getSimpleName());
            return;
        }

        Mutable<T> mutable = (Mutable<T>) values.get(enumClass);
        mutable.getInstance(currentState.name()).set(key, value);
    }

    /**
     * Modifies a value in a Mutable state instance.
     * @param <T> The type of the value being set.
     * @param enumClass The original enum class.
     * @param key The field name to update.
     * @param value The new value to set.
     * @param instanceName Optional. The name of the state to modify. If null, modifies the current state.
     */
    @SuppressWarnings("unchecked")
    public <T extends Comparable<T>> void modifyStateValue(Class<? extends Enum<?>> enumClass, String key, T value, String instanceName) {
        Mutable<T> mutable = (Mutable<T>) values.get(enumClass);
        if (mutable == null) {
            System.err.println("ERROR: No Mutable found for class " + enumClass.getSimpleName());
            return;
        }

        String stateName;
        if (instanceName == null) {
            System.err.println("WARNING: provided instanceName is null, defaulting to current state name");

            Enum<?> currentState = states.get(enumClass);
            if (currentState == null) {
                System.err.println("ERROR: No current state found for class: " + enumClass.getSimpleName());
                return;
            }
            stateName = currentState.name();
        } else {
            stateName = instanceName;
        }

        Mutable.Instance<T> instance = mutable.getInstance(stateName);
        if (instance == null) {
            System.err.println("ERROR: No instance found with name " + stateName);
            return;
        }

        instance.set(key, value);
    }

    /**
     * Adds a motor to the subsystem
     * @param name The name of the motor
     * @param motor The motor controller
     */
    protected void addMotor(String name, MotorController motor) {
        if (name == null || motor == null) {
            System.err.println("ERROR: Attempted to add null motor or name to " + getName());
            return;
        }

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

        if (name == null) {
            System.err.println("ERROR: Attempted to add PID values with null name to subsystem " + getName());
            return;
        }

        Optional<Double> kP = getClassFields(constants, "kP", "P");
        Optional<Double> kI = getClassFields(constants, "kI", "I");
        Optional<Double> kD = getClassFields(constants, "kD", "D");
        Optional<Double> kF = getClassFields(constants, "FF", "ff", "F", "kF");
        Optional<Double> kIZone = getClassFields(constants, "kIzone", "Izone", "kIz", "Iz", "IZ");
        Optional<Double> kIAccum = getClassFields(constants, "kIAccum", "IAccum", "Iacc", "Ia", "IA", "kIa");
        Optional<Double> kDFilter = getClassFields(constants, "kDFilter", "DFilter", "kDF", "kDF", "DF");
        Optional<Double> kMinOutput = getClassFields(constants, "kMinOutput", "MinOutput", "kMin");
        Optional<Double> kMaxOutput = getClassFields(constants, "kMaxOutput", "MaxOutput", "kMax");

        if (motor instanceof CANSparkBase) {
            SparkPIDController pidController = ((CANSparkBase) motor).getPIDController();
            
            kP.ifPresent(pidController::setP);
            kI.ifPresent(pidController::setI);
            kD.ifPresent(pidController::setD);
            kF.ifPresent(pidController::setFF);
            kIZone.ifPresent(pidController::setIAccum);
            kIAccum.ifPresent(pidController::setIAccum);
            kDFilter.ifPresent(pidController::setDFilter);
            kMinOutput.ifPresent(
                min -> kMaxOutput.ifPresent(
                    max -> pidController.setOutputRange(min, max)
                )
            );
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
    @SuppressWarnings("unchecked")
    private <T> Optional<T> getClassField(Class<?> clazz, String fieldName) {
        try {
            return Optional.ofNullable((T) clazz.getField(fieldName).get(null));
        } catch (Exception e) {
            return Optional.empty();
        }
    }

    /**
     * Helper function to get a double field from a possibility of multiple fields
     * @param clazz The static class
     * @param fieldNames The possible field names as arguments
     * @return A double value for the first valid field supplied as an argument
     */
    private <T> Optional<T> getClassFields(Class<?> clazz, String... fieldNames) {
        for (String fieldName : fieldNames) {
            Optional<T> field = getClassField(clazz, fieldName);
            if (field.isPresent()) {
                return field;
            }
        }
        return Optional.empty();
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

        if (motor == null) {
            System.err.println("ERROR: No motor found with name '" + motorName + "' in " + getName());
            return false;
        }

        if (motor instanceof CANSparkMax) {
            return Math.abs(((CANSparkMax) motor).getEncoder().getPosition() - targetPosition) < threshold;
        }

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