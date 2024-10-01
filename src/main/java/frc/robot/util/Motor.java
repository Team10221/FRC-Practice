package frc.robot.util;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class Motor {
  private int id;
  private PID pid;
  private String name;
  private MotorController motor;

  Motor() {

  }

  public int getID() {
    return id;
  }

  public PID getPID() {
    return pid;
  }

  public String getName() {
    return name;
  }

  public MotorController getMotor() {
    return motor;
  }
}
