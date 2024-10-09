package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Motor {
  private double threshold;

  private PID pid;
  private MotorAdapter adapter;
  private MotorController motor;

  public enum Control {
    POSITION, VELOCITY, VOLTAGE
  }

  private interface MotorAdapter {
    void setPID(PID pid);

    void setReference(double reference, Control controlType);

    double getPosition();
  }

  private Motor(MotorController motor, MotorAdapter adapter) {
    this.motor = motor;
    this.adapter = adapter;
  }

  public static Motor kraken(int id) {
    TalonFX kraken = new TalonFX(id);
    return new Motor(kraken, new TalonFXAdapter(kraken));
  }

  public static Motor kraken(int id, String CANBus) {
    TalonFX kraken = new TalonFX(id, CANBus);
    return new Motor(kraken, new TalonFXAdapter(kraken));
  }

  public static Motor falcon(int id) {
    TalonFX falcon = new TalonFX(id);
    return new Motor(falcon, new TalonFXAdapter(falcon));
  }

  public static Motor falcon(int id, String CANBus) {
    TalonFX falcon = new TalonFX(id, CANBus);
    return new Motor(falcon, new TalonFXAdapter(falcon));
  }

  public static Motor neo(int id) {
    CANSparkMax neo = new CANSparkMax(id, MotorType.kBrushless);
    return new Motor(neo, new SparkBaseAdapter(neo));
  }

  public void setPID(PID pid) {
    this.pid = pid;
    adapter.setPID(pid);
  }

  public void setReference(double reference, Control controlType) {
    adapter.setReference(reference, controlType);
  }

  public double getPosition() {
    return adapter.getPosition();
  }

  public boolean isAtTarget(double target) {
    return Math.abs(getPosition() - target) < threshold;
  }

  public void stop() {
    motor.stopMotor();
  }

  public void setThreshold(double threshold) {
    this.threshold = threshold;
  }

  public MotorController motor() {
    return motor;
  }

  public PID getPID() {
    return pid;
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
      return motor.getPosition().getValue();
    }
  }
}
