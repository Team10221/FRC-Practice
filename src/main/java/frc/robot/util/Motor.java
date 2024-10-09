package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
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
      ControlType type = switch (controlType) {
        case POSITION -> ControlType.kPosition;
        case VELOCITY -> ControlType.kVelocity;
        case VOLTAGE -> ControlType.kVoltage;
      };

      motor.getPIDController().setReference(reference, type);
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
      ControlRequest request = switch (controlType) {
        case POSITION -> new PositionVoltage(reference);
        case VELOCITY -> new VelocityVoltage(reference);
        case VOLTAGE -> new VoltageOut(reference);
      };

      motor.setControl(request);
    }

    public double getPosition() {
      return motor.getPosition().getValue();
    }
  }

  /*
   * public void addPIDValues() {
   * if (motor instanceof CANSparkBase) {
   * SparkPIDController pidController = ((CANSparkBase) motor).getPIDController();
   * pidController.setP(pid.getP());
   * pidController.setI(pid.getI());
   * pidController.setD(pid.getD());
   * } else if (motor instanceof TalonFX) {
   * TalonFXConfiguration config = new TalonFXConfiguration();
   * config.Slot0.kP = pid.getP();
   * config.Slot0.kI = pid.getI();
   * config.Slot0.kD = pid.getD();
   * ((TalonFX) motor).getConfigurator().apply(config);
   * }
   * }
   */

  /*
   * public void setReference(double reference, Control controlType) {
   * if (motor instanceof CANSparkBase) {
   * ((CANSparkBase) motor).getPIDController().setReference(reference,
   * controlType == Control.POSITION ? ControlType.kPosition
   * : controlType == Control.VELOCITY ? ControlType.kVelocity :
   * ControlType.kVoltage);
   * } else if (motor instanceof TalonFX) {
   * ((TalonFX) motor).setControl(
   * controlType == Control.POSITION ? new PositionVoltage(reference)
   * : controlType == Control.VELOCITY ? new VelocityVoltage(reference) : new
   * VoltageOut(reference));
   * }
   * }
   */

  /*
   * public double getPosition() {
   * if (motor instanceof CANSparkBase) {
   * return ((CANSparkBase) motor).getEncoder().getPosition();
   * } else if (motor instanceof TalonFX) {
   * return ((TalonFX) motor).getPosition().getValue();
   * }
   * 
   * return 0;
   * }
   */

  /*
   * public boolean isAtTarget(double target) {
   * if (motor instanceof CANSparkBase) {
   * return Math.abs(getPosition() - target) < threshold;
   * }
   * 
   * return false;
   * }
   */

  /*
   * public void setPID(PID pid) {
   * this.pid = pid;
   * }
   */
}
