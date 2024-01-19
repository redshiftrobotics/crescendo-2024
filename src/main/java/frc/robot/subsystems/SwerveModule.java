package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    private final double steeringOffset;

    /** Default state, forward and still */
    private final static SwerveModuleState defaultState = new SwerveModuleState();

    /** Represents the state the swerve module wants to be in */
    private SwerveModuleState desiredState;

    /** Locations of the wheel relative to the physical center of the robot. */
    private final Translation2d distanceFromCenter;

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
        driveMotor.setIdleMode(IdleMode.kBrake);

        // --- Drive Encoder ---
        driveEncoder = driveMotor.getEncoder();

        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.DRIVE_MOTOR_GEAR_RATIO);

        // --- Drive PID ---
        drivePIDController = driveMotor.getPIDController();
        drivePIDController.setP(SwerveModuleConstants.DRIVE_PID_P);
        drivePIDController.setI(SwerveModuleConstants.DRIVE_PID_I);
        drivePIDController.setD(SwerveModuleConstants.DRIVE_PID_D);
        drivePIDController.setFF(SwerveModuleConstants.DRIVE_PID_FF);
        drivePIDController.setIZone(SwerveModuleConstants.DRIVE_PID_IZone);
        drivePIDController.setOutputRange(-SwerveModuleConstants.MAX_SPEED_LIMIT, SwerveModuleConstants.MAX_SPEED_LIMIT);

        // --- Steering Motor ---
        steeringMotor = new CANSparkMax(steeringMotorDeviceId, MotorType.kBrushless);

        // --- Steering Encoder ---
        steeringEncoder = new CANcoder(steeringAbsoluteEncoderId);

        CANcoderConfiguration configuration = new CANcoderConfiguration();
        configuration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        steeringEncoder.getConfigurator().apply(configuration);
        
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
        steeringOffset = steeringEncoderZero;

        setName(toString());
    }

    /**
     * This is the periodic function of the swerve module.
     * This method is called periodically by the CommandScheduler, about every 20ms.
     */
    @Override
    public void periodic() {
        if (desiredState != null) { 
            // Get real and desired angles of steering motor
            // In pid the real value is often known as the measure or process variable, while the desired value is the setpoint or reference
            final double measuredRotations = getState().angle.getRotations();
            final double desiredRotations = desiredState.angle.getRotations();
            
            // Use the steering motor pid controller to calculate speed to turn steering motor to get to desired angle
            final double steeringMotorSpeed = steeringPIDController.calculate(measuredRotations, desiredRotations);
            // set steering motor to calculated value
            steeringMotor.set(steeringMotorSpeed);
    
            // the drive motor's PID controller is in RPM so we convert our value from Meters/Second to Rotations/Minute
            final double driveVelocityRotationsPerMinute = (desiredState.speedMetersPerSecond * 60) / SwerveModuleConstants.WHEEL_CIRCUMFERENCE;
            // Use the drive motors built in pid controller to reach target velocity
            drivePIDController.setReference(driveVelocityRotationsPerMinute, CANSparkMax.ControlType.kVelocity);
        }
    }

    /** Set desired state to nothing and stop drive and steering motor of swerve module. */
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
            state = SwerveModuleState.optimize(state,  getState().angle);
        }
        
        this.desiredState = state;
    }

    /**
     * Set the state of the swerve module. The state is the speed and angle of the swerve module.
     * You can use {@code Rotation2d.fromDegrees()} to create angle.
     * This version is meant for driving the robot, where a new state is set every 20ms.
     * It tries to make driving smoother by 
     * 
     * @param state New state of swerve module, contains speed in meters per second and angle as {@link Rotation2d}
     * @param shouldOptimize Whether to optimize the way the swerve module gets to the desired state
     */
    public void setDesiredStateDrive(SwerveModuleState state) {
        if (state != null) {

            // get current angle of robot
            Rotation2d encoderAngle = getState().angle;

            // Optimize the reference state to avoid spinning further than 90 degrees

            state = SwerveModuleState.optimize(state, encoderAngle);

            // Scale speed by cosine of angle error. 
            // This scales down movement perpendicular to the desired direction of travel that can occur when modules change directions.
            // This results in smoother driving.

            final double speed = state.speedMetersPerSecond;
            final double speedCosScaled = speed * state.angle.minus(encoderAngle).getCos();

            final double scaleSplit = SwerveModuleConstants.SWERVE_MODULE_DRIVE_COSIGN_SCALE;

            state.speedMetersPerSecond = (speed * scaleSplit) + (speedCosScaled * (1 - scaleSplit));
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
        return steeringPosition.refresh().getValueAsDouble() + steeringOffset;
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
