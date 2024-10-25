package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.command.ShortCommand;
import frc.robot.Constants.DeflectorConstants.DeflectorState;
import frc.robot.Constants.ShooterConstants.AngleState;
import frc.robot.commands.PositionForAmp;
import frc.robot.subsystems.Deflector;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  Deflector deflector = new Deflector();
  Shooter shooter = new Shooter();

  private final PS4Controller ps4Controller = new PS4Controller(0); // The parameter is the controller port

  private final JoystickButton l2Button = new JoystickButton(ps4Controller, PS4Controller.Button.kL2.value);
  private final JoystickButton r2Button = new JoystickButton(ps4Controller, PS4Controller.Button.kR2.value);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // examples
    r2Button.onTrue(new PositionForAmp(deflector, shooter));
    r2Button.onTrue(
        new InstantCommand(() -> {
          deflector.setState(DeflectorState.UP);
          shooter.setState(AngleState.UP);
        }, deflector, shooter));
    r2Button.onTrue(
        new ShortCommand(
            () -> {
              deflector.setState(DeflectorState.UP);
              shooter.setState(AngleState.UP);
            },
            () -> deflector.isAtTarget() && shooter.isAtTarget(),
            deflector, shooter));
    r2Button.onTrue(
        new ShortCommand(
            () -> deflector.setState(DeflectorState.UP),
            () -> deflector.isAtTarget(),
            deflector));
  }
}
