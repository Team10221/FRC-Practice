package frc.robot.subsystems;

import frc.lib.subsystem.Subsystem;
import frc.lib.subsystem.util.Motor;
import frc.robot.Constants.DeflectorConstants;
import frc.robot.Constants.DeflectorConstants.DeflectorState;

public class Deflector extends Subsystem {
    Motor deflectorAngle = Motor.neo(DeflectorConstants.DEFLECTOR_ID).setPID(DeflectorConstants.PID);

    public Deflector() {
        super(DeflectorState.class);
    }

    @Override
    protected void updateMotors() {
        deflectorAngle.setReference(getState(DeflectorState.class).position);
    }
}
