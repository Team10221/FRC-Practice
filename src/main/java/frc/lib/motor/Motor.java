package frc.lib.motor;

import java.util.function.Consumer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.lib.motor.adapters.SparkBaseAdapter;
import frc.lib.motor.adapters.TalonFXAdapter;
import frc.lib.util.PID;

/**
 * A wrapper class for motors, simplifying use and implementation.
 */
public class Motor {
  private double threshold;

  private PID pid;
  private MotorAdapter adapter;
  private MotorController motor;

  /**
   * An enum for common types of motor control: position, velocity, voltage.
   * Corresponds to each type of motor control for TalonFX and CANSparkBase motor
   * controllers.
   */
  public enum Control {
    POSITION, VELOCITY, VOLTAGE
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
    setThreshold(0.05);
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
   * Gets the name of the current motor controller.
   *
   * @return The current motor controller's name.
   */
  public String getType() {
    return motor.getClass().getName();
  }

  /**
   * Sets the speed of the motor, from -1 to 1.
   *
   * @param speed The speed to set
   * @return The motor object, allowing for method chaining.
   */
  public Motor set(double speed) {
    motor.set(speed);
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
   * Sets the motor's reference, assuming a position control.
   *
   * @param reference The reference value.
   * @return The motor object, allowing for method chaining.
   */
  public Motor setReference(double reference) {
    return setReference(reference, Control.POSITION);
  }

  /**
   * Sets the motor's (position/velocity) reference.
   * 
   * @param reference   The reference value.
   * @param controlType The type of reference, e.g. position or velocity.
   * @return The motor object, allowing for method chaining.
   */
  public Motor setRef(double reference, Control controlType) {
    return setRef(reference, controlType);
  }

  /**
   * Sets the motor's reference, assuming a position control.
   *
   * @param reference The reference value.
   * @return The motor object, allowing for method chaining.
   */
  public Motor setRef(double reference) {
    return setReference(reference);
  }

  /**
   * Use code based PID instead of integrated motor controller PID.
   * If no previously specified control type, POSITION is used.
   * This method is intended to be called in updateMotors, (like setReference) and
   * relies on being called frequently for the PID to controller recalculate.
   *
   * @param reference The reference value.
   * @return The motor object, allowing for method chaining.
   */
  public Motor setManualReference(double reference) {
    return setManualReference(reference, Control.POSITION);
  }

  /**
   * Use code based PID instead of integrated motor controller PID.
   * This method is intended to be called in updateMotors, (like setReference) and
   * relies on being called frequently for the PID controller to recalculate.
   *
   * @param reference The reference value.
   * @param control   The type of PID control (POSITION, VELOCITY, or VOLTAGE)
   * @return The motor object, allowing for method chaining.
   */
  public Motor setManualReference(double reference, Control controlType) {
    // TODO: FINISH
    /*
     * motor.set(pid.toPIDController().calculate(
     * switch (controlType) {
     * case POSITION -> getPosition();
     * case VELOCITY -> getVelocity();
     * case VOLTAGE -> getVoltage();
     * }, reference));
     */
    switch (controlType) {
      case VELOCITY -> {
        motor.set(pid.toPIDController().calculate(getVelocity(), reference));
      }
      case POSITION -> {
        // motor.set(pid.toPIDController().calculate(getPosition(), reference));
        // I'm not sure the above is correct?
      }
      case VOLTAGE -> {
        motor.setVoltage(pid.toPIDController().calculate(getVoltage(), reference));
      }
    }
    return this;
  }

  // TODO: Add javadoc comments for methods below

  public Motor setManRef(double reference) {
    return setManualReference(reference, Control.POSITION);
  }

  public Motor setManRef(double reference, Control controlType) {
    return setManualReference(reference, controlType);
  }

  public Motor setMRef(double reference) {
    return setManualReference(reference, Control.POSITION);
  }

  public Motor setMRef(double reference, Control controlType) {
    return setManualReference(reference, controlType);
  }

  /**
   * Returns the motor's current input voltage.
   * 
   * @return The motor's current input voltage.
   */
  public double getVoltage() {
    return adapter.getVoltage();
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
    return Math.abs(getPosition() - target) <= threshold;
  }

  /**
   * Sets a threshold for isAtTarget.
   * 
   * @param threshold The threshold to set.
   * @return The motor object, allowing for method chaining.
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
}
