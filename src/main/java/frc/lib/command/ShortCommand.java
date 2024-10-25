package frc.lib.command;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ShortCommand extends FunctionalCommand {
    public ShortCommand(Runnable toRun, BooleanSupplier isFinished, Subsystem... requirements) {
        super(toRun, () -> {}, interrupted -> {}, isFinished, requirements);
    }
}
