package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.DeflectorConstants;
import frc.robot.Constants.DeflectorConstants.AngleMotorPID;
import frc.robot.Constants.DeflectorConstants.DeflectorState;
import frc.robot.util.Subsystem;

public class Deflector extends Subsystem {
    public Deflector() {
        super(DeflectorState.class);
        addMotor("deflectorAngle", new CANSparkMax(DeflectorConstants.DEFLECTOR_ID, MotorType.kBrushless));
        addPIDValues("deflectorAngle", AngleMotorPID.class);
    }

    @Override
    protected void updateMotors() {
        setPIDReference("deflectorAngle", getState(DeflectorState.class).position, Control.POSITION);
    }
}
