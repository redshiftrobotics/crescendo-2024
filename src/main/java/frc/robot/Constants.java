package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Use Constants as convenient storage to hold robot-wide numerical or boolean
 * constants.
 * All constants should be visible globally (public), not be on the instance
 * (static), and should be not change (final).
 * Do not put any functionality in this class.
 */
public final class Constants {
	public static enum Bot {
		WOOD_BOT, PRACTICE_BOT, COMPETITION_BOT
	}

	public static final Bot currentBot;
	public static final String serialNumber;

	/**
	 * This code determines what bot is being deployed and sets constants accordingly.
	 * 
	 * Simulated bots cannot have a RoboRIO ID, so we must check if the bot is real.
	 * If it isn't, load production config.
	 * The production bot is always default, so if we do anything crazy to our bot during the tourney like switch the RoboRIO the code works.
	 * 
	 * @author Aceius E.
	 */
	static {
		serialNumber = RobotBase.isReal() ? RobotController.getSerialNumber() : "Simulation";

		switch (serialNumber) {
			case "03282B00": // Wood Bot Serial Number
				currentBot = Bot.WOOD_BOT;
				break;

			case "03238024": // Practice Bot Serial Number
				currentBot = Bot.PRACTICE_BOT;
				break;

			default: // Also use competition bot as default
				currentBot = Bot.COMPETITION_BOT;
				break;
		}
	}

	public static class DriverConstants {
		public static final int DRIVER_JOYSTICK_PORT = 0;

		public static final double DEAD_ZONE = 0.25;

		// Names of options for displaying
		public static final String[] maxSpeedOptionsNames = { "Precise", "Normal", "Boost" };

		// max forward/sideways velocities for drivetrain, in meters per second
		public static final double[] maxSpeedOptionsTranslation = { 0.1, 0.75, 1 };

		// max angular velocity for drivetrain, in radians per second
		public static final double[] maxSpeedOptionsRotation = { 0.1, 0.75, 1 };
	}

    public static class ArmConstants {

        public static final double MAXIMUM_ARM_DEGREES = 1;
        public static final double MINIMUM_ARM_DEGREES = 0;

        public static final double ARM_AMP_SHOOTING_DEGREES = 0;
        public static final double ARM_SPEAKER_SHOOTING_DEGREES = 0;
        public static final double ARM_INTAKE_DEGREES = 0;

        public static final int LEFT_MOTOR_ID = 0;
        //public static final int LEFT_ENCODER_ID = 0;
        public static final int RIGHT_MOTOR_ID = 0;
        public static final int RIGHT_ENCODER_ID = 0;

        public static final double DEGREES_PER_SECOND = 2.0;
    }
	public static class RobotMovementConstants {
		public static final double AT_SETPOINT_TOLERANCE_TIME_SECONDS = 1;
		public static final double ROTATE_AT_SETPOINT_TIME_SECONDS = 1;

		public static final double POSITION_TOLERANCE_METERS = Units.inchesToMeters(5);
		public static final double ANGLE_TOLERANCE_RADIANS = Units.degreesToRadians(5);

		public static final double ROTATION_PID_P = 0.5;
		public static final double ROTATION_PID_I = 0;
		public static final double ROTATION_PID_D = 0;

		public static final double TRANSLATION_PID_P = 75;
		public static final double TRANSLATION_PID_I = 1;
		public static final double TRANSLATION_PID_D = 0.5;
	}

	public static class OperatorConstants {
		public static final int OPERATOR_JOYSTICK_PORT = 1;
	}

	public static class SwerveModuleConstants {

		static {
			switch (currentBot) {
				case WOOD_BOT:
					// Front Left
					VELOCITY_MOTOR_ID_FL = 41;
					ANGULAR_MOTOR_ID_FL = 40;
					ANGULAR_MOTOR_ENCODER_ID_FL = 1;
					ANGULAR_MOTOR_ENCODER_OFFSET_FL = -0.611328125;

					// Front right
					VELOCITY_MOTOR_ID_FR = 4;
					ANGULAR_MOTOR_ID_FR = 5;
					ANGULAR_MOTOR_ENCODER_ID_FR = 2;
					ANGULAR_MOTOR_ENCODER_OFFSET_FR = -0.282958;

					// Back left
					VELOCITY_MOTOR_ID_BL = 3;
					ANGULAR_MOTOR_ID_BL = 2;
					ANGULAR_MOTOR_ENCODER_ID_BL = 4;
					ANGULAR_MOTOR_ENCODER_OFFSET_BL = -0.9260253906;

					// Back right
					VELOCITY_MOTOR_ID_BR = 42;
					ANGULAR_MOTOR_ID_BR = 6;
					ANGULAR_MOTOR_ENCODER_ID_BR = 3;
					ANGULAR_MOTOR_ENCODER_OFFSET_BR = -0.8641367187;
					break;

				case PRACTICE_BOT:
				default: // Temporary default to practice bot
					// Front Left
					VELOCITY_MOTOR_ID_FL = 2;
					ANGULAR_MOTOR_ID_FL = 3;
					ANGULAR_MOTOR_ENCODER_ID_FL = 3;
					ANGULAR_MOTOR_ENCODER_OFFSET_FL = -0.256015325670498;

					// Front right
					VELOCITY_MOTOR_ID_FR = 16;
					ANGULAR_MOTOR_ID_FR = 17;
					ANGULAR_MOTOR_ENCODER_ID_FR = 2;
					ANGULAR_MOTOR_ENCODER_OFFSET_FR = -0.248045977011494;

					// Back left
					VELOCITY_MOTOR_ID_BL = 8;
					ANGULAR_MOTOR_ID_BL = 9;
					ANGULAR_MOTOR_ENCODER_ID_BL = 4;
					ANGULAR_MOTOR_ENCODER_OFFSET_BL = -0.894674329501916;

					// Back right
					VELOCITY_MOTOR_ID_BR = 10;
					ANGULAR_MOTOR_ID_BR = 11;
					ANGULAR_MOTOR_ENCODER_ID_BR = 1;
					ANGULAR_MOTOR_ENCODER_OFFSET_BR = -0.530498084291188;
					break;

				// case COMPETITION_BOT:
				// default:

				// break;
			}
		}

		// Values from
		// https://www.swervedrivespecialties.com/products/mk4-swerve-module. We have L1
		// Modules.
		public static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 8.4;
		public static final double STEERING_MOTOR_GEAR_RATIO = 12.8;

		public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
		public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

		// Other settings
		public static final double MAX_SPEED_LIMIT = 1;
		public static final double SWERVE_MODULE_DRIVE_COSIGN_SCALE = 1;

		// Drive PID values
		public static final double DRIVE_PID_P = 0.000006;
		public static final double DRIVE_PID_I = 0.000001;
		public static final double DRIVE_PID_D = 0;
		public static final double DRIVE_PID_FF = 0.000015;
		public static final double DRIVE_PID_MAX_I = 0.001;

		// Steering PID values
		public static final double STEERING_PID_P = 0.5;
		public static final double STEERING_PID_I = 0;
		public static final double STEERING_PID_D = 0;

		// Front left
		public static final int VELOCITY_MOTOR_ID_FL;
		public static final int ANGULAR_MOTOR_ID_FL;
		public static final int ANGULAR_MOTOR_ENCODER_ID_FL;
		public static final double ANGULAR_MOTOR_ENCODER_OFFSET_FL;

		// Front right
		public static final int VELOCITY_MOTOR_ID_FR;
		public static final int ANGULAR_MOTOR_ID_FR;
		public static final int ANGULAR_MOTOR_ENCODER_ID_FR;
		public static final double ANGULAR_MOTOR_ENCODER_OFFSET_FR;

		// Back left
		public static final int VELOCITY_MOTOR_ID_BL;
		public static final int ANGULAR_MOTOR_ID_BL;
		public static final int ANGULAR_MOTOR_ENCODER_ID_BL;
		public static final double ANGULAR_MOTOR_ENCODER_OFFSET_BL;

		// Back right
		public static final int VELOCITY_MOTOR_ID_BR;
		public static final int ANGULAR_MOTOR_ID_BR;
		public static final int ANGULAR_MOTOR_ENCODER_ID_BR;
		public static final double ANGULAR_MOTOR_ENCODER_OFFSET_BR;
	}

	public static class SwerveDrivetrainConstants {
		static {
			switch (currentBot) {
				case WOOD_BOT:
					MODULE_LOCATION_X = 26.0 / 100;
					MODULE_LOCATION_Y = 28.5 / 100;
					break;

				case PRACTICE_BOT:
				default: // Temporary default to practice bot
					MODULE_LOCATION_X = 54 / 100;
					MODULE_LOCATION_Y = 54 / 100;
					break;

				// case COMPETITION_BOT:
				// default:

				// break;
			}
		}

		// distance of swerve modules from center of robot, in meters
		public static final double MODULE_LOCATION_Y;
		public static final double MODULE_LOCATION_X;
	}
}
