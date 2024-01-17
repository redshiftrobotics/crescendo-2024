package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * Use Constants as convenient storage to hold robot-wide numerical or boolean constants.
 * All constants should be visible globally (public), not be on the instance (static), and should be not change (final).
 * Do not put any functionality in this class.
 */
public final class Constants {

    public static class DriverConstants {
        public static final int DRIVER_JOYSTICK_PORT = 0;

        public static final double JOYSTICK_DEAD_ZONE = 0.15;

        // Names of options for displaying
        public static final String[] maxSpeedOptionsNames = {"Precise", "Normal", "Boost"};

        // max forward/sideways velocities for drivetrain, in meters per second
        public static final double[] maxSpeedOptionsTranslation = {0.44, 4.4, 11.6};
        
        // max angular velocity for drivetrain, in radians per second
        public static final double[] maxSpeedOptionsRotation = {1, 2, 4};
    }

    public static class OperatorConstants {
        public static final int OPERATOR_JOYSTICK_PORT = 1;
    }

    public static class SwerveModuleConstants {
        public static final double MAX_SPEED = 1;

        // Values from https://www.swervedrivespecialties.com/products/mk4-swerve-module. We have L1 Modules.
        public static final double DRIVE_MOTOR_GEAR_RATIO = 57 / 7;
        public static final double STEERING_MOTOR_GEAR_RATIO = 12.8;

        public static final double STEERING_ENCODER_SENSOR_COEFFICIENT = 0.000244140625; // if you put 1/4096 it just becomes zero

        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(7);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

        // Steering PID values
        public static final double STEERING_PID_P = 1;
        public static final double STEERING_PID_I = 0;
        public static final double STEERING_PID_D = 0.1;

        // Drive PID values
        public static final double DRIVE_PID_P = 1;
        public static final double DRIVE_PID_I = 0;
        public static final double DRIVE_PID_D = 0.1;
        public static final double DRIVE_PID_FF = 0;
        public static final double DRIVE_PID_IZone = 0;

        // Front left
		public static final int ANGULAR_MOTOR_ID_FL = 41;
		public static final int VELOCITY_MOTOR_ID_FL = 51;
		public static final int ANGULAR_MOTOR_ENCODER_ID_FL = 1;
		public static final double ANGULAR_MOTOR_ENCODER_OFFSET_FL = 0;
        
        // Front right
		public static final int ANGULAR_MOTOR_ID_FR = 4;
		public static final int VELOCITY_MOTOR_ID_FR = 52;
		public static final int ANGULAR_MOTOR_ENCODER_ID_FR = 2;
		public static final double ANGULAR_MOTOR_ENCODER_OFFSET_FR = 0;

        // Back left
		public static final int ANGULAR_MOTOR_ID_BL = 3;
		public static final int VELOCITY_MOTOR_ID_BL = 53;
		public static final int ANGULAR_MOTOR_ENCODER_ID_BL = 4;
		public static final double ANGULAR_MOTOR_ENCODER_OFFSET_BL = 0;
        
        // Back right
		public static final int ANGULAR_MOTOR_ID_BR = 42;
		public static final int VELOCITY_MOTOR_ID_BR = 54;
		public static final int ANGULAR_MOTOR_ENCODER_ID_BR = 3;
		public static final double ANGULAR_MOTOR_ENCODER_OFFSET_BR = 0;
    }

    public static class SwerveDrivetrainConstants {
        // distance of swerve modules from center of robot
        public static final double MODULE_LOCATION_X = 0.25;
        public static final double MODULE_LOCATION_Y = 0.25;
    }
}

