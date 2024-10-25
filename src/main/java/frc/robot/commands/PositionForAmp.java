package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DeflectorConstants.DeflectorState;
import frc.robot.Constants.ShooterConstants.AngleState;
import frc.robot.subsystems.Deflector;
import frc.robot.subsystems.Shooter;

public class PositionForAmp extends Command {
    private final Deflector deflector;
    private final Shooter shooter;
    
    public PositionForAmp(Deflector deflector, Shooter shooter) {
        addRequirements(deflector, shooter);
        this.deflector = deflector;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.setState(AngleState.UP);
        deflector.setState(DeflectorState.UP);
    }

    @Override
    public boolean isFinished() {
        return deflector.isAtTarget() && shooter.isAtTarget();
    }
}
