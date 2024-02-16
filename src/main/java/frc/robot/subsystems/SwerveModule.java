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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

/**
 * Subsystem for individual swerve module on robot. Each swerve module has one
 * drive motor and one steering motor.
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

	/** Locations of the wheel relative to the physical center of the robot. */
	private final Translation2d distanceFromCenter;

	/** Whether swerve module is stopped */
	private boolean stopped = false;

	/**
	 * Constructor for an individual Swerve Module.
	 * Sets up both drive and angular motor for swerve module as well as systems to
	 * monitor and control them
	 * 
	 * @param driveMotorDeviceId        device ID for drive motor
	 * @param steeringMotorDeviceId     device ID for steering motor
	 * @param steeringAbsoluteEncoderId device ID for the angular motor's absolute encoder
	 * @param distanceFromCenter        distance from center of robot to center of swerve module
	 * @param steeringEncoderZero       the zero (forward) position for the angular motor's absolute encoder, in rotations
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
		drivePIDController.setIAccum(SwerveModuleConstants.DRIVE_PID_MAX_I);
		drivePIDController.setOutputRange(-1, 1);

		// --- Steering Motor ---
		steeringMotor = new CANSparkMax(steeringMotorDeviceId, MotorType.kBrushless);

		steeringMotor.setIdleMode(IdleMode.kBrake);

		// --- Steering Encoder ---
		steeringEncoder = new CANcoder(steeringAbsoluteEncoderId);

		// Use 0-1 for rotational
		steeringEncoder.getConfigurator().apply(
				new MagnetSensorConfigs()
						.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
						.withMagnetOffset(steeringEncoderZero));

		// --- Steering PID ---
		steeringPIDController = new PIDController(
				SwerveModuleConstants.STEERING_PID_P,
				SwerveModuleConstants.STEERING_PID_I,
				SwerveModuleConstants.STEERING_PID_D);
		steeringPIDController.enableContinuousInput(0, 1);

		// --- Save other values ---
		this.distanceFromCenter = distanceFromCenter;

		steeringPosition = steeringEncoder.getAbsolutePosition();

		setName(toString());
	}

	/**
	 * This is the periodic function of the swerve module.
	 * This method is called periodically by the CommandScheduler, about every 20ms.
	 * 
	 * We use it to constantly keep the
	 */
	@Override
	public void periodic() {
		if (!stopped) {
			// Calculate how fast to spin the motor to get to the desired angle using our PID controller,
			// then set the motor to spin at that speed
			steeringMotor.set(steeringPIDController.calculate(getSteeringAngleRotations()));
		}
	}

	// --- Direct control methods ---

	/**
	 * Stop drive and steering motor of swerve module, module can be moved again by calling setDesiredState.
	 */
	public void stop() {
		stopped = true;

		// Manually stop both motors in swerve module
		driveMotor.stopMotor();
		steeringMotor.stopMotor();
	}

	/**
	 * Set the desired state of the Swerve Module to the default/starting state.
	 * This should have the module facing forward and not spinning.
	 */
	public void toDefaultState() {
		setDesiredState(defaultState, false);
	}

	/**
	 * Turn module to facing inward, directly towards center of robot, useful for braking
	 */
	public void toInwardPosition() {
		setDesiredState(new SwerveModuleState(
			0, distanceFromCenter.getAngle()
		), false);
	}

	// --- Getters and setters for modules desired SwerveModuleState ---

	/**
	 * Get the state of the swerve module.
	 * The state is the speed of our drive motor and angle of our steering motor.
	 * 
	 * @return Current state of swerve module, contains speed (in m/s) and angle as
	 *         {@link Rotation2d}
	 */
	public SwerveModuleState getState() {
		return new SwerveModuleState(
				getDriveSpeedMetersPerSecond(),
				Rotation2d.fromRotations(getSteeringAngleRotations()));
	}

	/**
	 * Set the state of the swerve module. The state is the speed and angle of the swerve module.
	 * You can use {@code Rotation2d.fromDegrees()} to create an angle.
	 * 
	 * @param state          New state of swerve module, contains speed in meters per second and angle as {@link Rotation2d}
	 * @param powerDriveMode Whether the SwerveModuleState is in meters per second (false) or motor power (true)
	 */
	public void setDesiredState(SwerveModuleState state, boolean powerDriveMode) {
		
		// If state is null, then stop robot and don't set any states
		if (state == null) {
			stop();
			return;
		}
		
		stopped = false;
		
		// Optimize the reference state to avoid spinning further than 90 degrees
		state = optimize(state, Rotation2d.fromRotations(getSteeringAngleRotations()));

		// --- Set steering motor ---
		steeringPIDController.setSetpoint(state.angle.getRotations());

		// --- Set drive motor ---

		if (state.speedMetersPerSecond == 0) {
			// If our desired speed is 0, just use the builtin motor stop, no matter the mode.

			// Stops motor movement. Motor can be moved again by calling set without having to re-enable the motor.
			driveMotor.stopMotor();
		} else if (powerDriveMode) {
			// If we are in power drive mode just directly set power to our desired speed.

			// This is a bit of an abuse of the SwerveModuleState object as we treat speeds as power values from -1 to 1.
			// We do this because we don't want to have to deal with a PID controller when we are just driving, as a human driver does not care about they exact speed mapping.
			driveMotor.set(state.speedMetersPerSecond);
		} else {
			// If we are in normal drive mode use the motor controller to set our target velocity.

			// The CANSparkMaxes have a builtin PID controller on them we can use to set a target velocity.
			// We first convert our speed from meters per second to rotations per minute, as that is the native unit of our devices
			final double desiredDriveRotationsPerMinute = (state.speedMetersPerSecond * 60) / SwerveModuleConstants.WHEEL_CIRCUMFERENCE;
			drivePIDController.setReference(desiredDriveRotationsPerMinute, ControlType.kVelocity);
		}
	}

	// --- Public info getters ---

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
	 * Get locations of the wheel relative to the physical center of the robot. Useful for kinematics.
	 * 
	 * @return Translation2d representing distance from center of bot
	 */
	public Translation2d getDistanceFromCenter() {
		return distanceFromCenter;
	}

	// --- Private getters ---

	/**
	 * Get the velocity of the drive motor.
	 * 
	 * @return Meters per second of the drive wheel
	 */
	private double getDriveSpeedMetersPerSecond() {
		return (SwerveModuleConstants.WHEEL_CIRCUMFERENCE * driveEncoder.getVelocity()) / 60;
	}

	/**
	 * Get the position of the drive motor.
	 * 
	 * @return Meters traveled by the drive wheel
	 */
	private double getDriveDistanceMeters() {
		return SwerveModuleConstants.WHEEL_CIRCUMFERENCE * driveEncoder.getPosition();
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
	 * to go a certain direction we can instead just turn a half as much and switch the speed of wheel to go in reverse.
	 * 
	 * @param desiredState The state you want the swerve module to be in
	 * @param currentAngle The current angle of the swerve module in degrees
	 * @return An optimized version of desiredState
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
	 * Utility method. Move an angle into the range of the reference. Finds the relative 0 and 360 position for a scope reference and moves the new angle into that.
	 * Example:
	 * <li>{@code placeInAppropriate0To360Scope(90, 370) = 10.0}, since 90 is in the 0-360 scope we move 370 into that scope
	 * <li>{@code placeInAppropriate0To360Scope(720, 10) = 730.0} since 720 is in the 720-1080 scope we move 10 to be in that scope
	 * 
	 * @param scopeReference The reference to find which 0-360 scope we are in.
	 * @param newAngle       The angle we want to move into found scope.
	 * @return {@code newAngle} in the same scope as {@code scopeReference}
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

		final String[] yPositions = { "Back", "Middle", "Front" };
		final String[] xPositions = { "Right", "Middle", "Left" };

		final int ySignum = (int) Math.signum(distanceFromCenter.getY());
		final int xSignum = (int) Math.signum(distanceFromCenter.getX());

		return xPositions[xSignum + 1] + yPositions[ySignum + 1] + "SwerveModule";
	}
}
