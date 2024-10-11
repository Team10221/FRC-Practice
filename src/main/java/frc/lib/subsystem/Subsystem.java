package frc.lib.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystem.util.Mutable;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstract base class for robot subsystems.
 */
public abstract class Subsystem extends SubsystemBase {
    private final Map<Class<? extends Enum<?>>, Map<Enum<?>, Map<Integer, Object>>> hooks;
    private final Map<Class<? extends Enum<?>>, Mutable<?>> values;
    private final Map<Class<? extends Enum<?>>, Enum<?>> states;

    /**
     * Constructs a Subsystem with initial states and values for the given enum
     * classes.
     * 
     * @param enumClasses The enum classes to initialize states and values from.
     */
    @SafeVarargs
    public Subsystem(Class<? extends Enum<?>>... enumClasses) {
        this.hooks = new HashMap<>();
        this.values = new HashMap<>();
        this.states = new HashMap<>();

        for (Class<? extends Enum<?>> clazz : enumClasses) {
            values.put(clazz, translate(clazz));
            states.put(clazz, clazz.getEnumConstants()[0]);
        }

        setName(this.getClass().getName().toLowerCase());
    }

    /**
     * Translates an enum class to a Mutable object.
     * 
     * @param <E>       The enum type.
     * @param <T>       The type of values in the enum (assumed to be the same for
     *                  all fields).
     * @param enumClass The enum class to translate.
     * @return A Mutable object representing the enum.
     */
    @SuppressWarnings("unchecked")
    public static <E extends Enum<E>, T> Mutable<T> translate(Class<? extends Enum<?>> enumClass) {
        Mutable<T> mutable = new Mutable<>();
        for (Enum<?> enumConstant : enumClass.getEnumConstants()) {
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
     * 
     * @param <E>        The enum type.
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
     * 
     * @param <E>   The enum type.
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
     * 
     * @param <T>       The type of the value to retrieve.
     * @param enumClass The enum class representing the state.
     * @return The value of the single field in the current state, or null if an
     *         error occurs.
     */
    @SuppressWarnings("unchecked")
    public <T> T getStateValue(Class<?> enumClass) {
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
            System.err.println("ERROR: Expected exactly one field, but found " + fieldNames.size() + " for state: "
                    + currentState.name());
            return null;
        }

        String singleFieldName = fieldNames.iterator().next();
        T value = instance.get(singleFieldName);
        if (value instanceof Supplier) {
            return ((Supplier<T>) value).get();
        }

        return value;
    }

    /**
     * Retrieves the value from the current state when there's only one field and
     * state enum.
     * 
     * @param <T> The type of the value to retrieve.
     * @return The value of the single field in the current state, or null if an
     *         error occurs.
     */
    public <T> T getStateValue() {
        if (values.size() != 1) {
            System.err.println("ERROR: Improper use for getStateValue. Exiting.");
            return null;
        }

        Class<? extends Enum<?>> enumClass = values.entrySet().iterator().next().getKey();
        return getStateValue(enumClass);
    }

    /**
     * Retrieves a specified value from the current state for an enum.
     * 
     * @param <T>       The enum type.
     * @param enumClass The original enum class.
     * @param key       The field name to retrieve.
     * @return The value of the specified field in the state.
     */
    @SuppressWarnings("unchecked")
    public <T> T getStateValue(Class<?> enumClass, String key) {
        Enum<?> currentState = states.get(enumClass);
        if (currentState == null) {
            System.err.println("ERROR: No current state found for class: " + enumClass.getSimpleName());
            return null;
        }

        Mutable<T> mutable = (Mutable<T>) values.get(enumClass);
        T value = mutable.getInstance(currentState.name()).get(key);
        if (value instanceof Supplier) {
            return ((Supplier<T>) value).get();
        }

        return value;
    }

    /**
     * Retrieves a specified value from the current state for a single state is
     * being used by the subsystem.
     * 
     * @param <T> The enum type.
     * @param key The field name to retrieve.
     * @return
     */
    public <T> T getStateValue(String key) {
        if (values.size() != 1) {
            System.err.println("Improper usage for getStateValue. Exiting.");
            return null;
        }

        Class<? extends Enum<?>> enumClass = values.entrySet().iterator().next().getKey();
        return getStateValue(enumClass, key);
    }

    /**
     * Retrieves a specified value from a mutable state instance.
     * 
     * @param <T>          The type of the value to retrieve.
     * @param enumClass    The original enum class.
     * @param key          The field name to retrieve.
     * @param instanceName Optional. The name of the state to retrieve. If null,
     *                     uses the current state.
     * @return The value of the specified field in the state.
     */
    @SuppressWarnings("unchecked")
    public <T> T getStateValue(Class<? extends Enum<?>> enumClass, String key, String instanceName) {
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

        T value = instance.get(key);
        if (value instanceof Supplier) {
            return ((Supplier<T>) value).get();
        }

        return value;
    }

    /**
     * Retrieves a specified value from a mutable state instance if only one state
     * enum is present.
     * 
     * @param <T>          The type of the value to retrieve.
     * @param key          The field name to retrieve.
     * @param instanceName Optional. The name of the state to retrieve. If null,
     *                     uses the current state.
     * @return The value of the specified field in the state.
     */
    public <T> T getStateValue(String key, String instanceName) {
        if (values.size() != 1) {
            System.err.println("ERROR: Improper use for getStateValue. Exiting.");
            return null;
        }

        Class<? extends Enum<?>> enumClass = values.entrySet().iterator().next().getKey();
        return getStateValue(enumClass, key, instanceName);
    }

    /**
     * Modifies the value of the current state when there's only one field.
     * 
     * @param <T>       The type of the value to set.
     * @param enumClass The enum class representing the state.
     * @param value     The new value to set for the single field.
     */
    @SuppressWarnings("unchecked")
    public <T> void modifyStateValue(Class<? extends Enum<?>> enumClass, T value) {
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
            System.err.println("ERROR: Expected exactly one field, but found " + fieldNames.size() + " for state: "
                    + currentState.name());
            return;
        }

        String singleFieldName = fieldNames.iterator().next();
        instance.set(singleFieldName, value);
    }

    /**
     * Modifies the value of the current state when there's only one field and one
     * state.
     * 
     * @param <T>   The type of the value to set.
     * @param value The new value to set for the single field.
     */
    @SuppressWarnings("unchecked")
    public <T> void modifyStateValue(T value) { // TODO: Rework approach
        if (values.size() != 1) {
            System.err.println("ERROR: Improper usage for modifyStateValue. Exiting");
            return;
        }

        Class<? extends Enum<?>> enumClass = values.entrySet().iterator().next().getKey();
        Enum<?> currentState = states.get(enumClass);

        if (currentState == null) {
            System.err.println("ERROR: No current state found for class: " + enumClass.getSimpleName());
            return;
        }

        Mutable<T> mutable = (Mutable<T>) values.entrySet().iterator().next().getValue();
        Mutable.Instance<T> instance = mutable.getInstance(currentState.name());

        if (instance == null) {
            System.err.print("ERROR: No instance found for state: " + currentState.name());
            return;
        }

        Set<String> fieldNames = instance.getKeys();
        if (fieldNames.size() != 1) {
            System.err.println("ERROR: Expected exactly one field, but found " + fieldNames.size() + " for state: "
                    + currentState.name());
            return;
        }

        String singleFieldName = fieldNames.iterator().next();
        instance.set(singleFieldName, value);
    }

    /**
     * Modifies a value of the current state.
     * 
     * @param <T>       The type of the value to set.
     * @param enumClass The enum class representing the state.
     * @param key       The field name.
     * @param value     The value to set.
     */
    @SuppressWarnings("unchecked")
    public <T> void modifyStateValue(Class<? extends Enum<?>> enumClass, String key, T value) { // TODO: Better error
                                                                                                // checking
        Enum<?> currentState = states.get(enumClass);

        if (currentState == null) {
            System.err.println("ERROR: No current state found for class: " + enumClass.getSimpleName());
            return;
        }

        Mutable<T> mutable = (Mutable<T>) values.get(enumClass);
        mutable.getInstance(currentState.name()).set(key, value);
    }

    /**
     * Modifies a value of the current state for when only one state is being used
     * by the subsytem.
     * 
     * @param <T>   The type of the value to set.
     * @param key   The field name.
     * @param value The value to set.
     */
    @SuppressWarnings("unchecked")
    public <T> void modifyStateValue(String key, T value) { // TODO: Better error checking
        if (values.size() != 1) {
            System.err.println("ERROR: Improper usage for modifyStateValue. Exiting");
            return;
        }

        Class<? extends Enum<?>> enumClass = values.entrySet().iterator().next().getKey();
        Enum<?> currentState = states.get(enumClass);

        if (currentState == null) {
            System.err.println("ERROR: No current state found for class: " + enumClass.getSimpleName());
            return;
        }

        Mutable<T> mutable = (Mutable<T>) values.entrySet().iterator().next().getValue();
        mutable.getInstance(currentState.name()).set(key, value);
    }

    /**
     * Modifies a value in a Mutable state instance.
     * 
     * @param <T>          The type of the value being set.
     * @param enumClass    The original enum class.
     * @param key          The field name to update.
     * @param value        The new value to set.
     * @param instanceName Optional. The name of the state to modify. If null,
     *                     modifies the current state.
     */
    @SuppressWarnings("unchecked")
    public <T> void modifyStateValue(Class<? extends Enum<?>> enumClass, String key, T value, String instanceName) {
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
     * Modifies a value in a Mutable state instance when only one instance is
     * present.
     * <p>
     * Do NOT use this if a subsystem uses multiple states.
     * 
     * @param <T>          The type of the value being set.
     * @param key          The field name to update.
     * @param value        The new value to set.
     * @param instanceName Optional. The name of the state to modify. If null,
     *                     modifies the current state.
     */
    @SuppressWarnings("unchecked")
    public <T> void modifyStateValue(String key, T value, String instanceName) {
        if (values.size() != 1) {
            System.err.println("ERROR: Improper usage for modifyStateValue. Exiting");
            return;
        }

        Class<? extends Enum<?>> enumClass = values.entrySet().iterator().next().getKey();
        Mutable<T> mutable = (Mutable<T>) values.entrySet().iterator().next().getValue();

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
     * Creates a hook for an object, linking it by reference.
     * For the enum constant provided, a hook is created for the first associated
     * value
     * 
     * @param enumConstant The enum constant to hook
     * @param value        The value to hook
     */
    protected void setHook(Enum<?> enumConstant, Object value) {
        hooks.computeIfAbsent(enumConstant.getDeclaringClass(), k -> new HashMap<>())
                .computeIfAbsent(enumConstant, k -> new HashMap<>())
                .put(0, value);
        updateHooks();
    }

    /**
     * Removes all hooks for a given enum constant
     * 
     * @param enumConstant The enum constant to remove hooks.
     */
    protected void removeHooks(Enum<?> enumConstant) {
        hooks.get(enumConstant.getDeclaringClass()).remove(enumConstant);
    }

    /**
     * Creates a hook for an object, linking it by reference.
     * 
     * @param enumConstant The enum constant to hook
     * @param value        The value to hook
     * @param index        The index of the value to hook
     */
    protected void setHook(Enum<?> enumConstant, Object value, Integer index) {
        hooks.computeIfAbsent(enumConstant.getDeclaringClass(), k -> new HashMap<>())
                .computeIfAbsent(enumConstant, k -> new HashMap<>())
                .put(index, value);
        updateHooks();
    }

    /**
     * Removes a hook from an enum constant.
     * 
     * @param enumConstant The enum constant to remove hooks from.
     * @param index        The hooked index to remove.
     */
    protected void removeHook(Enum<?> enumConstant, Integer index) {
        hooks.get(enumConstant.getDeclaringClass()).get(enumConstant).remove(index);
    }

    /**
     * An internal method to update values from hooks
     */
    private void updateHooks() { // TODO: Refactor maybe?
        for (Class<? extends Enum<?>> key : hooks.keySet()) {
            Map<Enum<?>, Map<Integer, Object>> map = hooks.get(key);
            for (Enum<?> enumConstant : map.keySet()) {
                Map<Integer, Object> hook = map.get(enumConstant);
                for (Integer idx : hook.keySet()) {
                    Object obj = hook.get(idx);
                    if (obj instanceof Double) {
                        modifyStateValue(enumConstant.getDeclaringClass(), enumConstant.name(), obj);
                    } else {
                        System.err.println("ERROR: Attempted to hook a non-double object");
                        return;
                    }
                }
            }
        }
    }

    /**
     * Abstract method to update motor outputs. Must be implemented by subclasses.
     */
    protected abstract void updateMotors();

    /**
     * Updates the SmartDashboard with current states and motor outputs.
     */
    protected void updateSmartDashboard() {
        states.forEach((stateClass, state) -> SmartDashboard.putString(getName() + " " + stateClass.getSimpleName(),
                state.toString()));
    }

    /**
     * Periodic method called repeatedly. Updates motors and SmartDashboard.
     */
    @Override
    public void periodic() {
        updateMotors();
        updateSmartDashboard();
    }
}
