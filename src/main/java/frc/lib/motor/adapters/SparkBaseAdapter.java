package frc.lib.motor.adapters;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import frc.lib.motor.MotorAdapter;
import frc.lib.motor.Motor.Control;
import frc.lib.util.PID;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;


public class SparkBaseAdapter implements MotorAdapter {
    private CANSparkBase motor;

    public SparkBaseAdapter(CANSparkBase motor) {
      this.motor = motor;
    }

    public void setPID(PID pid) {
      SparkPIDController pidController = motor.getPIDController();
      pidController.setP(pid.getP().orElse(0.0));
      pidController.setI(pid.getI().orElse(0.0));
      pidController.setD(pid.getD().orElse(0.0));
    }

    public void setReference(double reference, Control controlType) {
      motor.getPIDController().setReference(reference,
          switch (controlType) {
            case POSITION -> ControlType.kPosition;
            case VELOCITY -> ControlType.kVelocity;
            case VOLTAGE -> ControlType.kVoltage;
          });
    }

    public double getPosition() {
      return motor.getEncoder().getPosition();
    }

    public double getVelocity() {
      return motor.getEncoder().getVelocity();
    }

    public double getVoltage() {
      return motor.getBusVoltage() * motor.getAppliedOutput();
    }

    public void setInverted(boolean toInvert) {
      motor.setInverted(toInvert);
    }

    public boolean isInverted() {
      return motor.getInverted();
    }

    public void setCurrentLimit(double limit) {
      motor.setSmartCurrentLimit((int) limit);
    }

    public void setForwardLimit(double forward) {
      motor.setSoftLimit(SoftLimitDirection.kForward, (float) forward);
    }

    public void setBackLimit(double back) {
      motor.setSoftLimit(SoftLimitDirection.kReverse, (float) back);
    }

    public void setSoftLimits(double forward, double back) {
      setForwardLimit(forward);
      setBackLimit(back);
    }

    public void resetEncoder() {
      motor.getEncoder().setPosition(0);
    }
}