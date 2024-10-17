package frc.lib.subsystem.util;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;

public class PID {
  private Optional<Double> kP, kI, kD, kF, kIZone, kMaxOutput, kMinOutput, kDFilter;

  public PID(double kP, double kI, double kD) {
    this.kP = Optional.ofNullable(kP);
    this.kI = Optional.ofNullable(kI);
    this.kD = Optional.ofNullable(kD);
    this.kF = Optional.empty();
    this.kIZone = Optional.empty();
    this.kMaxOutput = Optional.empty();
    this.kMinOutput = Optional.empty();
    this.kDFilter = Optional.empty();
  }

  public PID(double kP, double kI, double kD, double kF) {
    this.kP = Optional.ofNullable(kP);
    this.kI = Optional.ofNullable(kI);
    this.kD = Optional.ofNullable(kD);
    this.kF = Optional.ofNullable(kF);
    this.kIZone = Optional.empty();
    this.kMaxOutput = Optional.empty();
    this.kMinOutput = Optional.empty();
    this.kDFilter = Optional.empty();
  }

  public PID() {
    this.kP = Optional.empty();
    this.kI = Optional.empty();
    this.kD = Optional.empty();
    this.kF = Optional.empty();
    this.kIZone = Optional.empty();
    this.kMaxOutput = Optional.empty();
    this.kMinOutput = Optional.empty();
    this.kDFilter = Optional.empty();
  }

  public PID(double kP, double kI, double kD, double kF,
      double kIZone, double kMaxOutput, double kMinOutput, double kDFilter) {
    this.kP = Optional.ofNullable(kP);
    this.kI = Optional.ofNullable(kI);
    this.kD = Optional.ofNullable(kD);
    this.kF = Optional.ofNullable(kF);
    this.kIZone = Optional.ofNullable(kIZone);
    this.kMaxOutput = Optional.ofNullable(kMaxOutput);
    this.kMinOutput = Optional.ofNullable(kMinOutput);
    this.kDFilter = Optional.ofNullable(kDFilter);
  }

  public boolean hasPID() {
    return kP.isPresent() && kI.isPresent() && kD.isPresent();
  }

  public PIDController toPIDController() {
    if (hasPID()) {
      return new PIDController(kP.get(), kI.get(), kD.get());
    } else {
      throw new Error("P, I, D constants not present.");
    }
  }

  public Optional<Double> getP() {
    return kP;
  }

  public Optional<Double> getI() {
    return kI;
  }

  public Optional<Double> getD() {
    return kD;
  }

  public Optional<Double> getF() {
    return kF;
  }

  public Optional<Double> getIZone() {
    return kIZone;
  }

  public Optional<Double> getMaxOutput() {
    return kMaxOutput;
  }

  public Optional<Double> getMinOutput() {
    return kMinOutput;
  }

  public Optional<Double> getDFilter() {
    return kDFilter;
  }

  public PID p(double kP) {
    this.kP = Optional.ofNullable(kP);
    return this;
  }

  public PID i(double kI) {
    this.kI = Optional.ofNullable(kI);
    return this;
  }

  public PID d(double kD) {
    this.kD = Optional.ofNullable(kD);
    return this;
  }

  public PID f(double kF) {
    this.kF = Optional.ofNullable(kF);
    return this;
  }

  public PID iZone(double kIZone) {
    this.kIZone = Optional.ofNullable(kIZone);
    return this;
  }

  public PID maxOut(double kMaxOutput) {
    this.kMaxOutput = Optional.ofNullable(kMaxOutput);
    return this;
  }

  public PID minOut(double kMinOutput) {
    this.kMinOutput = Optional.ofNullable(kMinOutput);
    return this;
  }

  public PID dFilter(double kDFilter) {
    this.kDFilter = Optional.ofNullable(kDFilter);
    return this;
  }
}
