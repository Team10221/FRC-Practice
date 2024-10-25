package frc.lib.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Mutable;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Abstract base class for robot subsystems.
 */
public abstract class Subsystem<T> extends SubsystemBase { 
    private final Map<Class<? extends Enum<?>>, Map<Enum<?>, Map<Integer, T>>> hooks;
    private final Map<Class<? extends Enum<?>>, Mutable<T>> values;
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
    // TODO: Review! can probably be redone, there shouldn't be type-casting.
    public static <E extends Enum<E>, T> Mutable<T> translate(Class<? extends Enum<?>> enumClass) {
        Mutable<T> mutable = new Mutable<>();
        for (Enum<?> enumConstant : enumClass.getEnumConstants()) {
            Mutable.Builder<T> builder = new Mutable.Builder<>(enumConstant.name());
            for (Field field : enumClass.getDeclaredFields()) {
                if (!field.isEnumConstant() && !field.isSynthetic()) {
                    field.setAccessible(true);
                    try {
                        builder.with(field.getName(), (T) field.get(enumConstant));
                    } catch (IllegalAccessException e) {
                        System.err.println("Error accessing field: " + field.getName()); 
                    }
                }
            }
            mutable.addInstance(builder.build());
        }
        return mutable;
    }

    private void validateSingleState() {
        if (values.size() != 1) {
            throw new IllegalStateException("Expected exactly 1 state but found " + values.size() + " states"); 
        }
    }

    private Class<? extends Enum<?>> getFirstStateClass() {
        return values.entrySet().iterator().next().getKey();
    }

    private String getCurrentState(Class<?> enumClass) {
        return states.get(enumClass).name();
    }

    private String getFirstKey(Class<? extends Enum<?>> enumClass) {
        return values.get(enumClass).getInstance(getCurrentState(enumClass)).getKeys().iterator().next();
    }

    /**
     * Gets the current state for a given enum class.
     * 
     * @param <E>        The enum type.
     * @param stateClass The class of the enum.
     * @return The current state, or the first enum constant if no state is set.
     */
    public <E extends Enum<E>> E getState(Class<E> stateClass) {
        return stateClass.cast(states.get(stateClass));
    }

    /**
     * Sets the state for a given enum class.
     * 
     * @param <E>   The enum type.
     * @param state The state to set.
     */
    public <E extends Enum<E>> void setState(E state) {
        states.put(state.getDeclaringClass(), state);
    }

    /**
     * Retrieves the value from the current state when there's only one field and
     * state enum.
     * 
     * @return The value of the single field in the current state, or null if an
     *         error occurs.
     */
    public T getStateValue() {
        return getStateValue(getFirstStateClass());
    }

    /**
     * Retrieves a specified value from the current state for a single state is
     * being used by the subsystem.
     * 
     * @param key The field name to retrieve.
     * @return
     */
    public T getStateValue(String key) {
        return getStateValue(getFirstStateClass(), key);
    }

    /**
     * Retrieves the value from the current state when there's only one field.
     * Alternatively, you could you use this to get the state value of the first field of an enum.
     * 
     * @param enumClass The enum class representing the state.
     * @return The value of the single field in the current state, or null if an
     *         error occurs.
     */
    public T getStateValue(Class<? extends Enum<?>> enumClass) {
        return getStateValue(enumClass, getFirstKey(enumClass));
    }

    /**
     * Retrieves a specified value from the current state for an enum.
     * 
     * @param enumClass The original enum class.
     * @param key       The field name to retrieve.
     * @return The value of the specified field in the state.
     */
    public T getStateValue(Class<? extends Enum<?>> enumClass, String key) {
        return getStateValue(enumClass, key, getCurrentState(enumClass));
    }

    /**
     * Retrieves a specified value from a mutable state instance if only one state
     * enum is present.
     * 
     * @param key          The field name to retrieve.
     * @param instanceName Optional. The name of the state to retrieve. If null,
     *                     uses the current state.
     * @return The value of the specified field in the state.
     */
    public T getStateValue(String key, String instanceName) {
        validateSingleState(); 
        return getStateValue(getFirstStateClass(), key, instanceName);
    }

    /**
     * Retrieves a specified value from a mutable state instance.
     * 
     * @param enumClass    The original enum class.
     * @param key          The field name to retrieve.
     * @param instanceName Optional. The name of the state to retrieve. If null,
     *                     uses the current state.
     * @return The value of the specified field in the state.
     */
    public T getStateValue(Class<? extends Enum<?>> enumClass, String key, String instanceName) {
        return values.get(enumClass).getInstance(instanceName).get(key);
    }

    /**
     * Modifies the value of the current state when there's only one field and one
     * state.
     * 
     * @param value The new value to set for the single field.
     */
    public void modifyStateValue(T value) {
        modifyStateValue(getFirstKey(getFirstStateClass()), value);
    }

    /**
     * Modifies a value of the current state for when only one state is being used
     * by the subsytem.
     * 
     * @param key   The field name.
     * @param value The value to set.
     */
    public void modifyStateValue(String key, T value) {
        modifyStateValue(getFirstStateClass(), key, value);
    }

    /**
     * Modifies a value of the current state.
     * 
     * @param enumClass The enum class representing the state.
     * @param key       The field name.
     * @param value     The value to set.
     */
    public void modifyStateValue(Class<? extends Enum<?>> enumClass, String key, T value) { 
        modifyStateValue(enumClass, key, value, getCurrentState(enumClass));
    }

    /**
     * Modifies the value of the current state when there's only one field.
     * 
     * @param enumClass The enum class representing the state.
     * @param value     The new value to set for the single field.
     */
    public void modifyStateValue(Class<? extends Enum<?>> enumClass, T value) {
        modifyStateValue(enumClass, getFirstKey(enumClass), value, getCurrentState(enumClass));
    }

    /**
     * Modifies a value in a Mutable state instance when only one state is present.
     * <p>
     * Do NOT use this if a subsystem uses multiple states.
     * 
     * @param key          The field name to update.
     * @param value        The new value to set.
     * @param instanceName Optional. The name of the state to modify. If null,
     *                     modifies the current state.
     */
    public void modifyStateValue(String key, T value, String instanceName) {
        validateSingleState();
        modifyStateValue(getFirstStateClass(), key, value, instanceName);
    }

    /**
     * Modifies a value in a Mutable state instance.
     * 
     * @param enumClass    The original enum class.
     * @param key          The field name to update.
     * @param value        The new value to set.
     * @param instanceName Optional. The name of the state to modify. If null,
     *                     modifies the current state.
     */
    public void modifyStateValue(Class<? extends Enum<?>> enumClass, String key, T value, String instanceName) {
        values.get(enumClass).getInstance(instanceName).set(key, value);
    }

    /**
     * Creates a hook for an object, linking it by reference.
     * For the enum constant provided, a hook is created for the first associated
     * value
     * 
     * @param enumConstant The enum constant to hook
     * @param value        The value to hook
     */
    protected void setHook(Enum<?> enumConstant, T value) {
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
    protected void setHook(Enum<?> enumConstant, T value, Integer index) {
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
    private void updateHooks() {
        hooks.forEach((key, map) -> {
            map.forEach((enumConstant, hook) -> {
                hook.forEach((idx, value) -> {
                    modifyStateValue(enumConstant.getDeclaringClass(), enumConstant.name(), hook.get(idx));
                });
            });
        });
    }

    /**
     * Abstract method to update motor outputs. Must be implemented by subclasses.
     */
    protected abstract void updateMotors();

    /**
     * Updates the SmartDashboard with current states and motor outputs.
     */
    protected void updateSmartDashboard() { // TODO: Review
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
