package frc.lib.subsystem.util;

import java.util.function.Consumer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * A wrapper class for motors, simplifying use and implementation.
 */
public class Motor {
  private double threshold;

  private PID pid;
  private MotorAdapter adapter;
  private MotorController motor;

  public enum Control {
    POSITION, VELOCITY, VOLTAGE
  }

  private interface MotorAdapter {
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

    boolean isInverted();

    default void setStatorCurrentLimit(double limit) {
    };
  }

  /**
   * Creates a Motor object given a motor and a motor adapter.
   * 
   * @param motor   The motor controller, either a talon fx or spark max/flex.
   * @param adapter The motor adapter, either SparkBaseAdapter or TalonFXAdapter.
   */
  private Motor(MotorController motor, MotorAdapter adapter) {
    this.motor = motor;
    this.adapter = adapter;
  }

  /**
   * Creates a kraken Motor without a specified CAN BUS.
   * 
   * @param id The motor's CAN ID.
   * @return The constructed motor object.
   */
  public static Motor kraken(int id) {
    TalonFX kraken = new TalonFX(id);
    return new Motor(kraken, new TalonFXAdapter(kraken));
  }

  /**
   * Creates a kraken Motor with a specified CAN BUS.
   * 
   * @param id     The motor's CAN ID.
   * @param CANBus The motor's CAN BUS.
   * @return The constructed motor object.
   */
  public static Motor kraken(int id, String CANBus) {
    TalonFX kraken = new TalonFX(id, CANBus);
    return new Motor(kraken, new TalonFXAdapter(kraken));
  }

  /**
   * Creates a falcon Motor without a specified CAN BUS.
   * 
   * @param id The motor's CAN ID.
   * @return The constructed motor object.
   */
  public static Motor falcon(int id) {
    TalonFX falcon = new TalonFX(id);
    return new Motor(falcon, new TalonFXAdapter(falcon));
  }

  /**
   * Creates a falcon Motor with a specified CAN BUS.
   * 
   * @param id     The motor's CAN ID.
   * @param CANBus The motor's CAN BUS.
   * @return The constructed motor object.
   */
  public static Motor falcon(int id, String CANBus) {
    TalonFX falcon = new TalonFX(id, CANBus);
    return new Motor(falcon, new TalonFXAdapter(falcon));
  }

  /**
   * Creates a neo Motor.
   * 
   * @param id The motor's CAN ID.
   * @return The constructed motor object.
   */
  public static Motor neo(int id) {
    CANSparkMax neo = new CANSparkMax(id, MotorType.kBrushless);
    return new Motor(neo, new SparkBaseAdapter(neo));
  }

  /**
   * Allows use of a lambda function to access and configure the motor controller
   * object directly.
   * 
   * @param config The lambda function, as a consumer.
   * @return The motor object, allowing for method chaining.
   */
  public Motor config(Consumer<MotorController> config) {
    config.accept(motor);
    return this;
  }

  /**
   * Sets the motor's PID.
   * 
   * @param pid The PID to set, in a PID object.
   * @return The motor object, allowing for method chaining.
   */
  public Motor setPID(PID pid) {
    this.pid = pid;
    adapter.setPID(pid);
    return this;
  }

  /**
   * Sets the motor's (position/velocity) reference.
   * 
   * @param reference   The reference value.
   * @param controlType The type of reference, e.g. position or velocity.
   * @return The motor object, allowing for method chaining.
   */
  public Motor setReference(double reference, Control controlType) {
    adapter.setReference(reference, controlType);
    return this;
  }

  /**
   * Returns the motor's current position.
   * 
   * @return The motor's current position.
   */
  public double getPosition() {
    return adapter.getPosition();
  }

  /**
   * Returns the motor's current velocity.
   *
   * @return The motor's current velocity.
   */
  public double getVelocity() {
    return adapter.getVelocity();
  }

  /**
   * Gets the motor's adapter.
   * 
   * @return The motor's adapter.
   */
  public MotorAdapter getAdapter() {
    return adapter;
  }

  /**
   * Sets the motor's inversion.
   * 
   * @param inverted Whether the motor should be inverted or not.
   * @return The motor object, allowing for method chaining.
   */
  public Motor setInverted(boolean inverted) {
    adapter.setInverted(inverted);
    return this;
  }

  /**
   * Inverts the motor, if not done so previously.
   * 
   * @return The motor object, allowing for method chaining.
   */
  public Motor invert() {
    if (!adapter.isInverted()) {
      adapter.setInverted(true);
    }
    return this;
  }

  /**
   * Sets a (supply) current limit on the motor.
   * 
   * @param val The current limit.
   * @return The motor object, allowing for method chaining.
   */
  public Motor setCurrentLimit(double val) {
    adapter.setCurrentLimit(val);
    return this;
  }

  /**
   * Sets a stator current limit on the motor.
   * 
   * @param val The stator current limit.
   * @return The motor object, allowing for method chaining.
   */
  public Motor setStatorCurrentLimit(double val) {
    adapter.setStatorCurrentLimit(val);
    return this;
  }

  /**
   * States whether the motor is at a specified target.
   * 
   * @param target The target.
   * @return Whether the motor is at the target or not.
   */
  public boolean isAtTarget(double target) {
    return Math.abs(getPosition() - target) < threshold;
  }

  /**
   * Sets a threshold for isAtTarget.
   * 
   * @param threshold The threshold to set.
   * @return Whether the motor is at the target or not.
   */
  public Motor setThreshold(double threshold) {
    this.threshold = threshold;
    return this;
  }

  /**
   * Gets the motor controller object itself.
   * 
   * @return The motor controller object.
   */
  public MotorController motor() {
    return motor;
  }

  /**
   * Gets the motor's PID.
   * 
   * @return The motor's PID.
   */
  public PID getPID() {
    return pid;
  }

  /**
   * Stops the motor.
   */
  public void stop() {
    motor.stopMotor();
  }

  public static class SparkBaseAdapter implements MotorAdapter {
    private CANSparkBase motor;

    SparkBaseAdapter(CANSparkBase motor) {
      this.motor = motor;
    }

    public void setPID(PID pid) {
      SparkPIDController pidController = motor.getPIDController();
      pidController.setP(pid.getP());
      pidController.setI(pid.getI());
      pidController.setD(pid.getD());
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

  private static class TalonFXAdapter implements MotorAdapter {
    private TalonFX motor;

    TalonFXAdapter(TalonFX motor) {
      this.motor = motor;
    }

    public void setPID(PID pid) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Slot0.kP = pid.getP();
      config.Slot0.kI = pid.getI();
      config.Slot0.kD = pid.getD();
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
}
