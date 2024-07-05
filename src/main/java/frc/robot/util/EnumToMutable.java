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
    public static <E extends Enum<E>, T extends Comparable<T>> Mutable<T> translate(Class<E> enumClass) {
        Mutable<T> mutable = new Mutable<>();

        for (E enumConstant: enumClass.getEnumConstants()) {
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
}