package frc.lib.motor;

import frc.lib.motor.Motor.Control;
import frc.lib.util.PID;

public interface MotorAdapter {
    void resetEncoder();
    void setPID(PID pid);
    void setInverted(boolean toInvert);
    void setCurrentLimit(double limit);
    void setReference(double reference, Control controlType);
    void setSoftLimits(double forward, double back);
    void setForwardLimit(double forward);
    void setBackLimit(double back);
    double getPosition();
    double getVelocity();
    double getVoltage();
    boolean isInverted();
    default void setStatorCurrentLimit(double limit) {};
}