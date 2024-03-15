package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
		WOOD_BOT,
		COMP_BOT,
		SIM_BOT
	}

	public static final Bot currentBot;
	public static final String serialNumber;

	/**
	 * This code determines what bot is being deployed and sets constants
	 * accordingly.
	 * 
	 * Simulated bots cannot have a RoboRIO ID, so we must check if the bot is real.
	 * If it isn't, load production config.
	 * The production bot is always default, so if we do anything crazy to our bot
	 * during the tourney like switch the RoboRIO the code works.
	 * 
	 * @author Aceius E.
	 */
	static {
		serialNumber = RobotBase.isReal() ? RobotController.getSerialNumber() : "simulation";

		switch (serialNumber) {
			case "simulation":
				currentBot = Bot.SIM_BOT;
				break;

			case "03282B00": // Wood Bot Serial Number
				currentBot = Bot.WOOD_BOT;
				break;

			case "03238024": // Practice (Now comp) Bot Serial Number
			default:
				currentBot = Bot.COMP_BOT;
				break;
		}
	}

	public static class HangConstants {
		static {
			switch (currentBot) {
				case WOOD_BOT:
					HAS_HANG = false;
					break;

				case COMP_BOT:
				default:
					HAS_HANG = true;
					break;
			}
		}

		public static final boolean HAS_HANG;

		public static final int LEFT_MOTOR_ID = 1;
		public static final int RIGHT_MOTOR_ID = 18;
		public static final int LEFT_LIMIT_SWITCH_ID = 1;
		public static final int RIGHT_LIMIT_SWITCH_ID = 2;

		public static final boolean LEFT_MOTOR_IS_INVERTED = false;
		public static final boolean RIGHT_MOTOR_IS_INVERTED = true;

		public static final double SPEED = 1.0;

	}

	public static class DriverConstants {
		public static final int DRIVER_JOYSTICK_PORT = 0;
		public static final int OPERATOR_JOYSTICK_PORT = 1;

		public static final double DEAD_ZONE = 0.07;

		public static final double SLEW_RATE_LIMIT_UP = 3.0;
		public static final double SLEW_RATE_LIMIT_DOWN = 3.0;

		public static final int NUMBER_OF_SPEED_OPTIONS = 4;

		// Names of options for displaying
		public static final String[] maxSpeedOptionsNames = { "Precise", "Normal", "Boost", "Boost++" };

		public static final double MAX_SPEED = SwerveDrivetrainConstants.MAX_OBTAINABLE_SPEED;
		public static final double MAX_ROTATION_SPEED = Math.PI;

		// max forward/sideways velocities for drivetrain, in meters per second
		public static final double[] maxSpeedOptionsTranslation = { 0.1 * MAX_SPEED, 0.5 * MAX_SPEED, 0.75 * MAX_SPEED,
				MAX_SPEED };

		// max angular velocity for drivetrain, in radians per second
		public static final double[] maxSpeedOptionsRotation = { 0.25 * MAX_ROTATION_SPEED, 0.5 * MAX_SPEED,
				0.75 * MAX_SPEED, MAX_SPEED };
	}

	public static class ArmConstants {
		static {
			switch (currentBot) {
				case WOOD_BOT:
					HAS_ARM = false;
					break;

				case COMP_BOT:
				default:
					HAS_ARM = true;
					break;
			}
		}

		public static final boolean HAS_ARM;

		public static final double MAXIMUM_ARM_DEGREES = 50.0;
		public static final double MINIMUM_ARM_DEGREES = -120.0;

		public static final double ARM_TOLERANCE_DEGREES = 2.0;

		// Degrees
		public static final double ARM_START_DEGREES = -38.0;
		public static final double ARM_STOW_DEGREES = -83.0;
		public static final double ARM_STOW_2_DEGREES = -100.0;
		public static final double ARM_AMP_SHOOTING_DEGREES = -27.8;
		public static final double ARM_SPEAKER_SHOOTING_DEGREES = -97.822 - 5 + 4 - 6;
		public static final double ARM_INTAKE_DEGREES = -109 - 1.5;

		public static final int LEFT_MOTOR_ID = 5;
		public static final int RIGHT_MOTOR_ID = 19;
		public static final int RIGHT_ENCODER_ID = 6;

		public static final boolean ARE_MOTORS_REVERSED = false;

		public static final double ELEVATION_PID_P = 6.5;
		public static final double ELEVATION_PID_I = 0.0;
		public static final double ELEVATION_PID_D = 0.0;
	}

	public static class IntakeShooterConstants {
		static {
			switch (currentBot) {
				case WOOD_BOT:
					HAS_INTAKE = false;
					break;

				case COMP_BOT:
				default:
					HAS_INTAKE = true;
					break;
			}
		}

		public static final boolean HAS_INTAKE;

		public static final boolean INTAKE_REVERSE = false;

		public static final int FLYWHEEL_MOTOR_1_ID = 13;
		public static final int FLYWHEEL_MOTOR_2_ID = 7;

		public static final int INTAKE_MOTOR_ID = 12;
		public static final int INTAKE_LIMIT_SWITCH_ID = -1;

		public static final double FLYWHEEL_SHOOTER_SPEED_AMP = 0.5;
		public static final double FLYWHEEL_SHOOTER_SPEED_SPEAKER = 1.0;

		public static final double INTAKE_GRABBER_SPEED_AMP = 1.0;
		public static final double INTAKE_GRABBER_SPEED_SPEAKER = 1.0;
		public static final double INTAKE_GRABBER_SPEED_INTAKE = -1.0;
	}

	public static class RobotMovementConstants {
		public static final double POSITION_TOLERANCE_METERS = Units.inchesToMeters(3);
		public static final double ANGLE_TOLERANCE_RADIANS = Units.degreesToRadians(1);

		public static final double ROTATION_PID_P = 0.21;
		public static final double ROTATION_PID_I = 0.001;
		public static final double ROTATION_PID_D = 0;

		public static final double TRANSLATION_PID_P = 0.25;
		public static final double TRANSLATION_PID_I = 0;
		public static final double TRANSLATION_PID_D = 0;
		public static final double MAX_TRANSLATION_SPEED = 1.0;
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
					ANGULAR_MOTOR_ENCODER_OFFSET_FL = -0.136474609375;

					// Front right
					VELOCITY_MOTOR_ID_FR = 4;
					ANGULAR_MOTOR_ID_FR = 5;
					ANGULAR_MOTOR_ENCODER_ID_FR = 2;
					ANGULAR_MOTOR_ENCODER_OFFSET_FR = -0.8828125 + 0.5;

					// Back left
					VELOCITY_MOTOR_ID_BL = 2;
					ANGULAR_MOTOR_ID_BL = 3;
					ANGULAR_MOTOR_ENCODER_ID_BL = 4;
					ANGULAR_MOTOR_ENCODER_OFFSET_BL = -0.517333984375 + 0.5;

					// Back right
					VELOCITY_MOTOR_ID_BR = 42;
					ANGULAR_MOTOR_ID_BR = 6;
					ANGULAR_MOTOR_ENCODER_ID_BR = 3;
					ANGULAR_MOTOR_ENCODER_OFFSET_BR = -0.52001953125 + 0.5;
					break;

				case COMP_BOT:
				default:
					// Front Left
					VELOCITY_MOTOR_ID_FL = 2;
					ANGULAR_MOTOR_ID_FL = 3;
					ANGULAR_MOTOR_ENCODER_ID_FL = 3;
					ANGULAR_MOTOR_ENCODER_OFFSET_FL = -0.881591796875 + 0.5;

					// Front right
					VELOCITY_MOTOR_ID_FR = 14; // Was 16
					ANGULAR_MOTOR_ID_FR = 17;
					ANGULAR_MOTOR_ENCODER_ID_FR = 4;
					ANGULAR_MOTOR_ENCODER_OFFSET_FR = -0.881591796875;

					// Back left
					VELOCITY_MOTOR_ID_BL = 8;
					ANGULAR_MOTOR_ID_BL = 9;
					ANGULAR_MOTOR_ENCODER_ID_BL = 2;
					ANGULAR_MOTOR_ENCODER_OFFSET_BL = -0.641357421875;

					// Back right
					VELOCITY_MOTOR_ID_BR = 10;
					ANGULAR_MOTOR_ID_BR = 11;
					ANGULAR_MOTOR_ENCODER_ID_BR = 1;
					ANGULAR_MOTOR_ENCODER_OFFSET_BR = -0.549072265625;
					break;
			}
		}

		// Values from
		// https://www.swervedrivespecialties.com/products/mk4-swerve-module. We have L1
		// Modules.
		public static final double DRIVE_MOTOR_GEAR_RATIO = 1.0 / 8.4;
		public static final double STEERING_MOTOR_GEAR_RATIO = 12.8;

		public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
		public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

		// Other settings
		public static final double MAX_SPEED_LIMIT = 1.0;
		public static final double SWERVE_MODULE_DRIVE_COSIGN_SCALE = 1.0;

		// Drive PID values
		public static final double DRIVE_PID_P = 0.000006;
		public static final double DRIVE_PID_I = 0.000000;
		public static final double DRIVE_PID_D = 0.0;
		public static final double DRIVE_PID_FF = 0.15;
		public static final double DRIVE_PID_MAX_I = 0.001;

		// Steering PID values
		public static final double STEERING_PID_P = 1.5;
		public static final double STEERING_PID_I = 0.0;
		public static final double STEERING_PID_D = 0.0;

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

				case COMP_BOT:
				default:
					MODULE_LOCATION_X = 54.0 / 100.0;
					MODULE_LOCATION_Y = 54.0 / 100.0;
					break;
			}
		}

		// distance of swerve modules from center of robot, in meters
		public static final double MODULE_LOCATION_Y;
		public static final double MODULE_LOCATION_X;

		public static final double MAX_OBTAINABLE_SPEED = 2.0; // 3 Meters per Second
	}

	public static class VisionConstants {
		static {
			switch (currentBot) {
				case WOOD_BOT:
					CAMERA_NAME = "Arducam_OV2311_USB_Camera";
					break;
				case COMP_BOT:
				default:
					CAMERA_NAME = "Arducam_OV9281_USB_Camera";
					break;
			}
		}
		public static final String CAMERA_NAME;

		public static final Transform3d CAMERA_POSE = new Transform3d(Units.inchesToMeters(13), 0,
				Units.inchesToMeters(11), new Rotation3d(0, Units.degreesToRadians(-20), 0));
		public static final Transform3d ROBOT_TO_FRONT = new Transform3d(Units.inchesToMeters(-13), 0, 0,
				new Rotation3d());
	}

	public static class LightConstants {
		public static final int LED_CONTROLLER_PWM_SLOT = 0; // TODO find real value for this
		public static final int LED_QUANTITY = 60;

		public static final double LED_COLOR_RED = 0.61;
		public static final double LED_COLOR_ORANGE = 0.65;
		public static final double LED_COLOR_YELLOW = 0.69;
		public static final double LED_COLOR_GREEN = 0.77;
		public static final double LED_COLOR_BLUE = 0.87;
		public static final double LED_COLOR_PURPLE = 0.91;
		public static final double LED_COLOR_WHITE = 0.93;
		public static final double LED_COLOR_RAINBOW = -0.99;

		public static final double LED_COLOR_DEFAULT = 0.99;
	}
}
