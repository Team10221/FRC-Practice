package frc.robot;

import frc.lib.util.PID;

public final class Constants {
	public static class IntakeConstants {
		public static final int ANGLE_MOTOR_ID = 0;
		public static final int FEEDER_MOTOR_ID = 1;
		public static final int INTAKE_MOTOR_ID = 2;
		public static final double ANGLE_DOWN_POSITION = 0.0;
		public static final double ANGLE_UP_POSITION = 0.0;
		public static final double INTAKE_SPEED = 0.0;
		public static final double OUTTAKE_SPEED = 0.0;
		public static final double FEED_SPEED = 0.0;
		public static final double ACCURACY_THRESHOLD = 0.1;

		public static final PID PID = new PID(0, 0, 0);

		public enum IntakeState {
			IDLE(0.0, 0.0, IntakeConstants.ANGLE_UP_POSITION),
			INTAKE(IntakeConstants.INTAKE_SPEED, IntakeConstants.FEED_SPEED, IntakeConstants.ANGLE_DOWN_POSITION),
			OUTTAKE(IntakeConstants.OUTTAKE_SPEED, -IntakeConstants.FEED_SPEED, IntakeConstants.ANGLE_DOWN_POSITION);

			public final double intakeSpeed, feederSpeed, anglePosition;

			IntakeState(double intakeSpeed, double feederSpeed, double anglePosition) {
				this.intakeSpeed = intakeSpeed;
				this.feederSpeed = feederSpeed;
				this.anglePosition = anglePosition;
			}
		}
	}

	public static class ShooterConstants {
		public static final int SHOOTER_MOTOR_UP_ID = 3;
		public static final int SHOOTER_MOTOR_DOWN_ID = 4;
		public static final int ANGLE_MOTOR_ID = 5;
		public static final double ACCURACY_THRESHOLD = 0.1;
		public static final double SHOOTER_SPEED = 1.0;

		public static final PID PID = new PID(0, 0, 0);

		public static enum AngleState {
			RESTING(0.0),
			UP(0.0);

			public final double position;

			AngleState(double position) {
				this.position = position;
			}
		}

		public static enum ShooterState {
			IDLE(0.0, 0.0),
			SHOOTING(SHOOTER_SPEED, -SHOOTER_SPEED),
			INVERSE(-SHOOTER_SPEED, SHOOTER_SPEED),
			REVERSE(-0.25, -0.25);

			public final double topSpeed, bottomSpeed;

			ShooterState(double topSpeed, double bottomSpeed) {
				this.topSpeed = topSpeed;
				this.bottomSpeed = bottomSpeed;
			}
		}
	}

	public static class DeflectorConstants {
		public static final int DEFLECTOR_ID = 6;
		public static final PID PID = new PID(0, 0, 0);

		public static enum DeflectorState {
			UP(0.0),
			DOWN(0.0);

			public final double position;

			DeflectorState(double position) {
				this.position = position;
			}
		}
	}
}
