package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

/**
 * Subsystem for individual swerve module on robot. Each swerve module has one drive motor and one steering motor.
 * 
 * @see <a href="https://www.swervedrivespecialties.com/products/mk4-swerve-module">Swerve Module Kit</a>
 */
public class SwerveModule extends SubsystemBase {

    // the drive motor is the motor that spins the wheel making the robot move across the ground (aka wheel or velocity motor)
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkPIDController drivePIDController;
   
    // the steering motor is the motor that changes the rotation of the wheel allowing the robot to drive in any direction (aka spin or angular motor)
    // Also allows for spinning in place
    private final CANSparkMax steeringMotor;
    private final CANcoder steeringEncoder;
    private final PIDController steeringPIDController;

    /** Status Signal that gives steering encoders current position in rotations */
    private final StatusSignal<Double> steeringPosition;

    /** Default state, forward and still */
    private final static SwerveModuleState defaultState = new SwerveModuleState();

    /** Represents the state the swerve module wants to be in */
    private SwerveModuleState desiredState;

    /** Locations of the wheel relative to the physical center of the robot. */
    private final Translation2d distanceFromCenter;

    private boolean powerDriveMode = false;

    /**
     * Constructor for an individual Swerve Module.
     * Sets up both drive and angular motor for swerve module as well as systems to monitor and control them
     * 
     * @param velocityMotorDeviceID  device ID for drive motor
     * @param steeringMotorDeviceId  device ID for steering motor
     * @param angularEncoderDeviceID device ID for the angular motor's absolute encoder
     * @param distanceFromCenter     distance from center of robot to center of swerve module
     * @param steeringEncoderZero    the zero (forward) position for the angular motor's absolute encoder, in rotations
     */
    public SwerveModule(int driveMotorDeviceId, int steeringMotorDeviceId, int steeringAbsoluteEncoderId, double steeringEncoderZero, Translation2d distanceFromCenter) {
        // --- Drive Motor ---
        driveMotor = new CANSparkMax(driveMotorDeviceId, MotorType.kBrushless);

        // You most restore factory defaults if you want to use velocity encoder.
        // If you do not do this, everything will break and shake itself to death
        driveMotor.restoreFactoryDefaults();

        driveMotor.setIdleMode(IdleMode.kBrake);

        // --- Drive Encoder ---
        driveEncoder = driveMotor.getEncoder();

        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.DRIVE_MOTOR_GEAR_RATIO);
        driveEncoder.setPositionConversionFactor(SwerveModuleConstants.DRIVE_MOTOR_GEAR_RATIO);

        // --- Drive PID ---
        drivePIDController = driveMotor.getPIDController();
        drivePIDController.setP(SwerveModuleConstants.DRIVE_PID_P);
        drivePIDController.setI(SwerveModuleConstants.DRIVE_PID_I);
        drivePIDController.setD(SwerveModuleConstants.DRIVE_PID_D);
        drivePIDController.setFF(SwerveModuleConstants.DRIVE_PID_FF);
        drivePIDController.setOutputRange(-1, 1);

        // --- Steering Motor ---
        steeringMotor = new CANSparkMax(steeringMotorDeviceId, MotorType.kBrushless);

        // --- Steering Encoder ---
        steeringEncoder = new CANcoder(steeringAbsoluteEncoderId);

        // Use 0-1 for rotational
        steeringEncoder.getConfigurator().apply(
            new MagnetSensorConfigs()
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withMagnetOffset(steeringEncoderZero)
        );
        
        // --- Steering PID ---
        steeringPIDController = new PIDController(
            SwerveModuleConstants.STEERING_PID_P,
            SwerveModuleConstants.STEERING_PID_I,
            SwerveModuleConstants.STEERING_PID_D
        );
        steeringPIDController.enableContinuousInput(0, 1);
        
        // --- Save other values ---
        this.distanceFromCenter = distanceFromCenter;

        steeringPosition = steeringEncoder.getAbsolutePosition();

        setName(toString());
    }

    /**
     * This is the periodic function of the swerve module.
     * This method is called periodically by the CommandScheduler, about every 20ms.
     */
    @Override
    public void periodic() {
        if (desiredState != null) { 

            // --- Set drive motor ---

            SmartDashboard.putBoolean("powerDriveMode", powerDriveMode);
            
            if (desiredState.speedMetersPerSecond == 0) {
                // If our desired speed is 0, just use the built in motor stop, no matter the mode.
                driveMotor.stopMotor();
            }
            else if (powerDriveMode) {
                // If we are in power drive mode just directly set power to our desired speed.
                // This is a bit of an abuse of the SwerveModuleState object but we don't want to have to deal with a pid controller when we are just driving
                driveMotor.set(desiredState.speedMetersPerSecond);
            }
            else {
                // If we are  in normal drive mode use our drive motor builtin PID controller in velocity mode to set it to our desired meters per second
                final double desiredDriveRotationsPerMinute = (desiredState.speedMetersPerSecond * 60) / SwerveModuleConstants.WHEEL_CIRCUMFERENCE;
                drivePIDController.setReference(desiredDriveRotationsPerMinute, ControlType.kVelocity);
            }

            // --- Set steering motor ---

            // get our current angle and our desired angle
            final double desiredRotations = desiredState.angle.getRotations();
            final double measuredRotations = getSteeringAngleRotations();

            // calculate how fast to spin the motor to get to the desired angle using our PID controller
            final double steeringMotorSpeed = steeringPIDController.calculate(measuredRotations, desiredRotations);
            steeringMotor.set(steeringMotorSpeed);
        }
    }

    /** stop drive and steering motor of swerve module and set desired state to nothing*/
    public void stop() {

        // Make sure we have no desired state
        setDesiredState(null);

        // Manually stop both swerve modules
        driveMotor.stopMotor();
        steeringMotor.stopMotor();
    }

    /** 
     * Set the desired state of the Swerve Module to the default/starting state.
     * This should have the module facing forward and not spinning.
     */
    public void toDefaultState() {
        setDesiredState(defaultState);
    }

    /** Get locations of the wheel relative to the physical center of the robot. */
    public Translation2d getDistanceFromCenter() {
        return distanceFromCenter;
    }

    /**
     * Get the state of the swerve module.
     * The state is the speed of our drive motor and angle of our steering motor.
     * 
     * @return Current state of swerve module, contains speed (in m/s) and angle as {@link Rotation2d}
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveSpeedMetersPerSecond(),
                Rotation2d.fromRotations(getSteeringAngleRotations()));
    }

    public void enablePowerDriveMode() {
        this.powerDriveMode = true;
    }

    public void disablePowerDriveMode() {
        this.powerDriveMode = false;
    }

    /**
     * Set the state of the swerve module. The state is the speed and angle of the swerve module.
     * You can use {@code Rotation2d.fromDegrees()} to create angle.
     * 
     * @param state New state of swerve module, contains speed in meters per second and angle as {@link Rotation2d}
     * @param shouldOptimize Whether to optimize the way the swerve module gets to the desired state
     */
    public void setDesiredState(SwerveModuleState state, boolean shouldOptimize) {
        if (shouldOptimize && state != null) {
            // Optimize the reference state to avoid spinning further than 90 degrees
            state = optimize(state,  getState().angle);
        }
        
        this.desiredState = state;
    }

    /**
     * Set the state of the swerve module. The state is the speed and angle of the swerve module.
     * You can use {@code Rotation2d.fromDegrees()} to create angle.
     * This version is meant for driving the robot, where a new state is set every 20ms.
     * It tries to make driving smoother by scaling speed by the cosine of the angle
     * It also expects states to be in power mode, meaning instead of meters per second the motors are givin the direct speed to spin at
     * 
     * @param state New state of swerve module, contains speed in meters per second and angle as {@link Rotation2d}
     * @param shouldOptimize Whether to optimize the way the swerve module gets to the desired state
     */
    public void setDesiredStateDrive(SwerveModuleState state) {
        if (state != null) {

            // get current angle of robot
            Rotation2d encoderAngle = getState().angle;

            // Optimize the reference state to avoid spinning further than 90 degrees
            state = optimize(state, encoderAngle);

            // Scale speed by cosine of angle error. 
            // This scales down movement perpendicular to the desired direction of travel that can occur when modules change directions.
            // This results in smoother driving.
            final double speed = state.speedMetersPerSecond;
            final double speedCosScaled = speed * state.angle.minus(encoderAngle).getCos();

            final double scaleSplit = SwerveModuleConstants.SWERVE_MODULE_DRIVE_COSIGN_SCALE;

            state.speedMetersPerSecond = (speed * (1 - scaleSplit)) + (speedCosScaled * scaleSplit);
        }
        
        this.desiredState = state;
    }

    /**
     * Set the state of the swerve module. Will automatically optimize.
     * The state is the speed and angle of the swerve module.
     * You can use {@code Rotation2d.fromDegrees()} to create angle.
     * 
     * @param state New state of swerve module, contains speed in meters per second and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        setDesiredState(state, true);
    }

    /**
     * Get the desired state of the swerve module. The state is the speed and angle of the swerve module.
     * 
     * @return State that the swerve module is trying to achieve, contains speed in meters per second and angle.
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Get the position of the swerve module. The position is the distance traveled by the drive motor and angle of the drive motor.
     * 
     * @return Current position of swerve module, contains distance traveled by motor in meters and angle as {@link Rotation2d}
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDriveDistanceMeters(),
                Rotation2d.fromRotations(getSteeringAngleRotations()));
    }

    /**
     * Get the velocity of the drive motor.
     * 
     * @return Rotations per minute of the drive motor
     */
    private double getDriveSpeedRotationsPerMinute() {
        return driveEncoder.getVelocity();
    }

    /**
     * Get the velocity of the drive motor.
     * 
     * @return Meters per second of the drive wheel
     */
    private double getDriveSpeedMetersPerSecond() {
        return (SwerveModuleConstants.WHEEL_CIRCUMFERENCE * getDriveSpeedRotationsPerMinute()) / 60;
    }

    /**
     * Get the position of the drive motor.
     * 
     * @return Meters traveled by the drive wheel
     */
    private double getDriveDistanceMeters() {
        return SwerveModuleConstants.WHEEL_CIRCUMFERENCE * getDriveDistanceRotations();
    }

    /**
     * Get the position of the drive motor.
     * 
     * @return Total number of rotations of the drive motor
     */
    private double getDriveDistanceRotations() {
        return driveEncoder.getPosition();
    }

    /**
     * Get the angel of the steering motor.
     * 
     * @return Current position in rotations of the steering motor, accounting for offset
     */
    private double getSteeringAngleRotations() {
        return steeringPosition.refresh().getValueAsDouble();
    }

    // --- Util ---

    /**
     * Optimize a swerve module state so that instead of suddenly rotating the wheel (with steering motor)
     * to go a certain direction we can instead just turn a half as much and switch
     * the speed of wheel to go in reverse.
     * 
     * @param desiredState The state you want the swerve module to be in
     * @param currentAngle The current angle of the swerve module in degrees
     * @return             An optimized version of desiredState
     */
    private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        // find the target angle in the same 0-360 scope as the desired state
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());

        // keep the same target speed
        double targetSpeed = desiredState.speedMetersPerSecond;

        // found how much we have to move to get to target angle
        double delta = targetAngle - currentAngle.getDegrees();

        // If we have to flip around more than 90 degrees than instead just reverse our direction
        // and only turn enough so that we have the motor facing in the same direction, just the other way 
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle += delta > 0 ? -180 : 180;
        }

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * Utility method. Move an angle into the range of the reference. Finds the relative 0 and 360
     * position for a scope reference and moves the new angle into that.
     * Example: {@code placeInAppropriate0To360Scope(90, 370) = 10.0}
     * {@code placeInAppropriate0To360Scope(720, 10) = 730.0}
     * 
     * @param scopeReference The reference to find which 0-360 scope we are in.
     *                       For example {@code 10} is in {@code 0-360} scope while {@code 370} is in {@code 360-720} scope.
     * @param newAngle       The angle we want to move into found scope.
     *                       For example if the scope was {@code 0-360} and our angle was {@code 370} it would become {@code 10}
     * @return               {@code newAngle} in the same scope as {@code scopeReference}
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    @Override
    public String toString() {
        
        final String[] yPositions = {"Back", "Middle", "Front"}; 
        final String[] xPositions = {"Right", "Middle", "Left"};

        final int ySignum = (int) Math.signum(distanceFromCenter.getY());
        final int xSignum = (int) Math.signum(distanceFromCenter.getX());
        
        return xPositions[xSignum + 1] + yPositions[ySignum + 1] + "SwerveModule";
    }
}
