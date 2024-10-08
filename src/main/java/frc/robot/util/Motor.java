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
  private String CANbus;
  private MotorController motor;

  public enum Control {
    POSITION,
    VELOCITY,
    VOLTAGE
  }

  private Motor(int id) {
    motor = new CANSparkMax(id, MotorType.kBrushless);
  }

  private Motor(int id, String CANbus) {
    motor = new TalonFX(id, CANbus);
  }

  public static Motor kraken(int id, String CANbus) {
    return new Motor(id, CANbus);
  }

  public static Motor neo(int id) {
    return new Motor(id);
  }

  public static Motor falcon(int id, String CANbus) {
    return new Motor(id, CANbus);
  }

  public void addPIDValues() {
    if (motor instanceof CANSparkBase) {
      SparkPIDController pidController = ((CANSparkBase) motor).getPIDController();
      pidController.setP(pid.getP());
      pidController.setI(pid.getI());
      pidController.setD(pid.getD());
    } else if (motor instanceof TalonFX) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      config.Slot0.kP = pid.getP();
      config.Slot0.kI = pid.getI();
      config.Slot0.kD = pid.getD();
      ((TalonFX) motor).getConfigurator().apply(config);
    }
  }

  public void setReference(double reference, Control controlType) {
    if (motor instanceof CANSparkBase) {
      ((CANSparkBase) motor).getPIDController().setReference(reference,
          controlType == Control.POSITION ? ControlType.kPosition
              : controlType == Control.VELOCITY ? ControlType.kVelocity : ControlType.kVoltage);
    } else if (motor instanceof TalonFX) {
      ((TalonFX) motor).setControl(
          controlType == Control.POSITION ? new PositionVoltage(reference)
              : controlType == Control.VELOCITY ? new VelocityVoltage(reference) : new VoltageOut(reference));
    }
  }

  public double getPosition() {
    if (motor instanceof CANSparkBase) {
      return ((CANSparkBase) motor).getEncoder().getPosition();
    } else if (motor instanceof TalonFX) {
      return ((TalonFX) motor).getPosition().getValue();
    }

    return 0;
  }

  public boolean isAtTarget(double target) {
    if (motor instanceof CANSparkBase) {
      return Math.abs(getPosition() - target) < threshold;
    }

    return false;
  }

  public void stop() {
    motor.stopMotor();
  }

  public void setPID(PID pid) {
    this.pid = pid;
  }

  public void setThreshold(double threshold) {
    this.threshold = threshold;
  }

  public PID getPID() {
    return pid;
  }

  public MotorController motor() {
    return motor;
  }
}
