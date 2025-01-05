package frc.lib.motor.adapters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.motor.MotorAdapter;
import frc.lib.motor.Motor.Control;
import frc.lib.util.PID;

public class TalonFXAdapter implements MotorAdapter {
    private TalonFX motor;

    public TalonFXAdapter(TalonFX motor) {
      this.motor = motor;
    }

    public void setPID(PID pid) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Slot0.kP = pid.getP().orElse(0.0);
      config.Slot0.kI = pid.getI().orElse(0.0);
      config.Slot0.kD = pid.getD().orElse(0.0);
      motor.getConfigurator().apply(config);
    }

    public void setReference(double reference, Control controlType) {
      motor.setControl(
          switch (controlType) {
            case POSITION -> new PositionVoltage(reference);
            case VELOCITY -> new VelocityVoltage(reference);
            case VOLTAGE -> new VoltageOut(reference);
          });
    }

    public double getPosition() {
      return motor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
      return motor.getVelocity().getValueAsDouble();
    }

    public double getVoltage() {
      return motor.getMotorVoltage().getValueAsDouble();
    }

    public void setInverted(boolean toInvert) {
      motor.setInverted(toInvert);
    }

    public boolean isInverted() {
      return motor.getInverted();
    }

    public void setCurrentLimit(double limit) {
      CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
      currentConfigs.SupplyCurrentLimit = limit;
      currentConfigs.SupplyCurrentLimitEnable = true;
      motor.getConfigurator().apply(currentConfigs);
    }

    public void setStatorCurrentLimit(double limit) {
      CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
      currentConfigs.StatorCurrentLimit = limit;
      currentConfigs.StatorCurrentLimitEnable = true;
      motor.getConfigurator().apply(currentConfigs);
    }

    public void setForwardLimit(double forward) {
      SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
      configs.ForwardSoftLimitThreshold = forward;
      configs.ForwardSoftLimitEnable = true;
      motor.getConfigurator().apply(configs);
    }

    public void setBackLimit(double back) {
      SoftwareLimitSwitchConfigs configs = new SoftwareLimitSwitchConfigs();
      configs.ReverseSoftLimitThreshold = back;
      configs.ReverseSoftLimitEnable = true;
      motor.getConfigurator().apply(configs);
    }

    public void setSoftLimits(double forward, double back) {
      setForwardLimit(forward);
      setBackLimit(back);
    }

    public void resetEncoder() {
      motor.setPosition(0);
    }
  }