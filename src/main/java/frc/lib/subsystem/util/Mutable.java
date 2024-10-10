package frc.lib.subsystem.util;

import java.util.Collection;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/**
 * A mutable data structure that mimics enum-like behavior.
 */
public class Mutable<T> {

    /**
     * Builder class for creating Instance objects.
     */
    public static class Builder<T> {
        private final String name;
        private final Map<String, T> values = new ConcurrentHashMap<>();

        /**
         * Constructs a new Builder with a given name.
         * @param name The name of the Instance to be built. 
         */
        public Builder(String name) {
            this.name = name;
        }

        /**
         * Adds a key-value pair to the instance being built.
         * @param key The key for the value.
         * @param value The value associated with the key.
         * @return The Builder instance for method chaining.
         */
        public Builder<T> with(String key, T value) {
            values.put(key, value);
            return this;
        }

        /**
         * Builds and returns a new Instance with the current name and values.
         * @return A new Instance object.
         */
        public Instance<T> build() {
            return new Instance<>(name, values);
        }
    }

    /**
     * A class representing a single instance within the Mutable, similar to an enum constant.
     */
    public static class Instance<T> {
        private final String name;
        private final Map<String, T> values;
        
        private Instance(String name, Map<String, T> values) {
            this.name = name;
            this.values = new ConcurrentHashMap<>(values);
        }

        /**
         * Retries the value associated with a given key.
         * @param key The key whose associated value is to be returned.
         * @return The value associated with the specified key.
         */
        public T get(String key) {
            return values.get(key);
        }

        /**
         * Associates the specified value with the specified key.
         * @param key The key with which the specified value is to be associated.
         * @param value The value to be associated with the specified key.
         */
        public void set(String key, T value) {
            values.put(key, value);
        }

        /**
         * Gets the keys from this Instance and returns it as a Set.
         * @return A set object of the keys contained in this Instance.
         */
        public Set<String> getKeys() {
            return values.keySet();
        }

        /**
         * Gets the values from this Instance and returns it as a Collection.
         * @return A Collection object of the values contained in this Instance.
         */
        public Collection<T> values() {
            return values.values();
        }

        /**
         * Gets the name of this Instance as a string.
         * @return The name of this Instance.
         */
        public String getName() {
            return name;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof Instance)) return false;
            Instance<?> that = (Instance<?>) o;
            return Objects.equals(name, that.name) && Objects.equals(values, that.values);
        }

        @Override
        public int hashCode() {
            return Objects.hash(name, values);
        }

        @Override
        public String toString() {
            return name + values;
        }
    }

    private final Map<String, Instance<T>> instances = new ConcurrentHashMap<>();

    /**
     * Adds an Instance to this Mutable.
     * @param instance The instance to be added.
     */
    public void addInstance(Instance<T> instance) {
        instances.put(instance.getName(), instance);
    }

    /**
     * Retrieves an Instance by its name.
     * @param name The name of the Instance to retrieve.
     * @return The Instance with the specified name, or null if not found.
     */
    public Instance<T> getInstance(String name) {
        return instances.get(name);
    }

    /**
     * Gets the Instances from this Mutable and returns it as a Collection.
     * @return A Collection object of the Instances in this Mutable.
     */
    public Collection<Instance<T>> values() {
        return instances.values();
    }

    /**
     * Gets the number of Instances within this Mutable.
     * @return The number of Instances in this Mutable.
     */
    public int size() {
        return instances.size();
    }

    /**
     * Checks if this Mutable contains an Instance with the specified name.
     * @param name The name to check for.
     * @return true if this Mutable contains an Instance with the specified name, false otherwise
     */
    public boolean containsInstance(String name) {
        return instances.containsKey(name);
    }
}