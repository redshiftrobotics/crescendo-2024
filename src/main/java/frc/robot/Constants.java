package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * Use Constants as convenient to hold robot-wide numerical or boolean constants.
 * Everything here should be accessible everywhere (public), only exist once (static), and not change (final).
 */
public final class Constants {

    public static final class DriverConstants {
        public static final int DRIVER_JOYSTICK_PORT = 0;
    }

    public static final class OperatorConstants {
        public static final int OPERATOR_JOYSTICK_PORT = 1;
    }

    public static class SwerveModuleConstants {
        public static final double MAX_SPEED = 1;
        public static final double DRIVE_PID_P = 1;
        public static final double DRIVE_PID_I = 0.1;
        public static final double DRIVE_PID_D = 1;
        public static final double DRIVE_PID_FF = 0.01;
        public static final double DRIVE_PID_IZone = 0;
        public static final double DRIVE_PID_IAccum = 1000;

        public static final double STEERING_PID_P = 1;
        public static final double STEERING_PID_I = 0.1;
        public static final double STEERING_PID_D = 1;
        // public static final double STEERING_PID_FF = 0.01;
        // public static final double STEERING_PID_IZone = 0;
        // public static final double STEERING_PID_IAccum = 1000;

        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(7);
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    }
}
