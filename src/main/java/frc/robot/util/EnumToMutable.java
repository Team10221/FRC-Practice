package frc.robot.util;

import java.lang.reflect.Field;

public class EnumToMutable {

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

        // Iterate over all enum constants
        for (Enum<?> enumConstant: enumClass.getEnumConstants()) {
            Mutable.Builder<T> builder = new Mutable.Builder<>(enumConstant.name());

            // Iterate over all enum fields
            for (Field field : enumClass.getDeclaredFields()) {
                // Skip enum constants and synthetic fields
                if (!field.isEnumConstant() && !field.isSynthetic()) {
                    // Access fields independent of their access modifier
                    field.setAccessible(true);

                    // Extract field value
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
}