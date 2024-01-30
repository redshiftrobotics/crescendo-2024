package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Use Constants as convenient storage to hold robot-wide numerical or boolean constants.
 * All constants should be visible globally (public), not be on the instance (static), and should be not change (final).
 * Do not put any functionality in this class.
 */
public final class Constants {
    // Bot-specific constants.
    public static final String SOME_VALUE; // This is a demo/test constant

    /**
     * This code determines what bot is being deployed and sets constants accordingly.
     * 
     * Simulated bots cannot have a RoboRIO ID, so we must check if the bot is real. If it isn't, load production config. 
     * The production bot is always default, so if we do anything crazy to our bot during the tourney like swich the roborio the code works.
     * 
     * @author Aceius E.
     */
    static {
        String serialNumber = RobotBase.isReal() ? RobotController.getSerialNumber() : "default";

        switch (serialNumber) {
            case "03282B00": // Wood Bot
                SOME_VALUE = "Wood Bot Value";
                break;
        
            default: // Production Bot
                SOME_VALUE = "Comp Bot Value";
                break;
        }
    }

    public static class DriverConstants {
        public static final int DRIVER_JOYSTICK_PORT = 0;

        public static final double JOYSTICK_DEAD_ZONE = 0.25;
        public static final double XBOX_DEAD_ZONE = 0.10;
        public static final double PS4_DEAD_ZONE = 0.12;

        // Names of options for displaying
        public static final String[] maxSpeedOptionsNames = {"Precise", "Normal", "Boost"};

        // max forward/sideways velocities for drivetrain, in meters per second
        public static final double[] maxSpeedOptionsTranslation = {0.1, 0.75, 1};
        
        // max angular velocity for drivetrain, in radians per second
        public static final double[] maxSpeedOptionsRotation = {0.1, 0.75, 1};
    }
    
    public static class RobotMovementConstants {
        public static final double maxSpeed = 1;
        public static final double maxTurnSpeed = 1;

        public static final double MOVE_PID_TOLERANCE_TIME = 500 / 20;
        public static final double ROTATE_PID_TOLERANCE_TIME = 500 / 20;

        public static final double POS_TOLERANCE = 0.1;
        public static final double ANGLE_TOLERANCE = Math.PI / 30;

        public static final double ROTATION_PID_P = 0.5;
        public static final double ROTATION_PID_I = 0;
        public static final double ROTATION_PID_D = 0;

        public static final double TRANSLATION_PID_P = 5;
        public static final double TRANSLATION_PID_I = 0;
        public static final double TRANSLATION_PID_D = 0;
    }

    public static class OperatorConstants {
        public static final int OPERATOR_JOYSTICK_PORT = 1;
    }

    public static class SwerveModuleConstants {
        
        // Values from https://www.swervedrivespecialties.com/products/mk4-swerve-module. We have L1 Modules.
        public static final double DRIVE_MOTOR_GEAR_RATIO = 8.4;
        public static final double STEERING_MOTOR_GEAR_RATIO = 12.8;
                
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;
        
        // Other settings
        public static final double MAX_SPEED_LIMIT = 1;
        public static final double SWERVE_MODULE_DRIVE_COSIGN_SCALE = 1;

        // Drive PID values
        public static final double DRIVE_PID_P = 0.000006;
        public static final double DRIVE_PID_I = 0;
        public static final double DRIVE_PID_D = 0;
        public static final double DRIVE_PID_FF = 0.000015;

        // Steering PID values
        public static final double STEERING_PID_P = 0.5;
        public static final double STEERING_PID_I = 0;
        public static final double STEERING_PID_D = 0;

        // Front left
		public static final int VELOCITY_MOTOR_ID_FL = 41;
		public static final int ANGULAR_MOTOR_ID_FL = 40;
		public static final int ANGULAR_MOTOR_ENCODER_ID_FL = 1;
		public static final double ANGULAR_MOTOR_ENCODER_OFFSET_FL = -0.611328125;
        
        // Front right
		public static final int VELOCITY_MOTOR_ID_FR = 4;
		public static final int ANGULAR_MOTOR_ID_FR = 5;
		public static final int ANGULAR_MOTOR_ENCODER_ID_FR = 2;
		public static final double ANGULAR_MOTOR_ENCODER_OFFSET_FR = -0.282958;

        // Back left
		public static final int VELOCITY_MOTOR_ID_BL = 3;
		public static final int ANGULAR_MOTOR_ID_BL = 2;
		public static final int ANGULAR_MOTOR_ENCODER_ID_BL = 4;
		public static final double ANGULAR_MOTOR_ENCODER_OFFSET_BL = -0.9260253906;
        
        // Back right
		public static final int VELOCITY_MOTOR_ID_BR = 42;
		public static final int ANGULAR_MOTOR_ID_BR = 6;
		public static final int ANGULAR_MOTOR_ENCODER_ID_BR = 3;
		public static final double ANGULAR_MOTOR_ENCODER_OFFSET_BR = -0.8641367187;
    }

    public static class SwerveDrivetrainConstants {
        // distance of swerve modules from center of robot, in meters
        public static final double MODULE_LOCATION_Y = 28.5 / 100;
        public static final double MODULE_LOCATION_X = 26.0 / 100;
    }
}

