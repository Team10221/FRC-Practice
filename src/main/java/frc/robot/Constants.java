package frc.robot;

public final class Constants {
	public static class IntakeConstants {
		public static final int ANGLE_MOTOR_ID = 0;
		public static final int FEEDER_MOTOR_ID = 1;
		public static final int INTAKE_MOTOR_ID = 2;

		public static final double ANGLE_MOTOR_P = 0.0;
		public static final double ANGLE_MOTOR_I = 0.0;
		public static final double ANGLE_MOTOR_D = 0.0;

		public static final double ANGLE_DOWN_POSITION = 0.0;
		public static final double ANGLE_UP_POSITION = 0.0;

		public static final double INTAKE_SPEED = 0.0;
		public static final double OUTTAKE_SPEED = 0.0;
		public static final double FEED_SPEED = 0.0;
	}

	public static class ShooterConstants {
		public static final int SHOOTER_MOTOR_UP_ID = 3;
		public static final int SHOOTER_MOTOR_DOWN_ID = 4;
		public static final int ANGLE_MOTOR_ID = 5;

		public static final double ANGLE_PID_P = 0.1;
		public static final double ANGLE_PID_I = 0.0;
		public static final double ANGLE_PID_D = 0.0;

		public static final double SHOOTER_SPEED_UP = 1.0;
		public static final double SHOOTER_SPEED_DOWN = 1.0;

		public static enum AngleState {
			UP(0.0),
			MIDDLE(0.0),
			DOWN(0.0);

			public final double position;

			AngleState(double position) {
				this.position = position;
			}
		}

		public static enum ShooterState {
			ON,
			OFF;
		}
	}
}