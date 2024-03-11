package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriverConstants;


/** Class that stores supplies for main controls of ChassisSpeeds */
public class ChassisDriveInputs extends SubsystemBase {

	private final Supplier<Double> xSupplier, ySupplier, rotationSupplier;

	private final double xCoefficient, yCoefficient, rotationCoefficient;

	private final SlewRateLimiter xSlewRateLimiter, ySlewRateLimiter, rotationSlewRateLimiter;

	private final double deadzone;

	private int speedLevel = DriverConstants.NUMBER_OF_SPEED_OPTIONS / 2;
	private boolean isFieldRelative = false;

	/**
	 * Create a new ChassisDriveInputs
	 * 
	 * @param getForward          Get the value mapped to X, -1 full backward to +1
	 *                            full forward
	 * @param forwardCoefficient  Coefficient that forward (X) multiplied by
	 * 
	 * @param getLeft             Get the value mapped to Y, -1 full right to +1
	 *                            full left
	 * @param leftCoefficient     Coefficient that forward left (Y) are multiplied
	 *                            by
	 * 
	 * @param getRotation         Get the value mapped to rotation, -1 full clock
	 * @param rotationCoefficient Coefficient that rotation is multiplied by
	 * 
	 * @param deadzone            Deadzone for all axises
	 */
	public ChassisDriveInputs(
			Supplier<Double> getForward, double forwardCoefficient,
			Supplier<Double> getLeft, double leftCoefficient,
			Supplier<Double> getRotation, double rotationCoefficient,
			double deadzone, int rateLimitUp, double rateLimitDown) {

		this.ySupplier = getForward;
		this.xSupplier = getLeft;
		this.rotationSupplier = getRotation;

		this.yCoefficient = forwardCoefficient;
		this.xCoefficient = leftCoefficient;
		this.rotationCoefficient = rotationCoefficient;

		this.xSlewRateLimiter = new SlewRateLimiter(rateLimitUp, -rateLimitDown, 0);
		this.ySlewRateLimiter = new SlewRateLimiter(rateLimitUp, -rateLimitDown, 0);
		this.rotationSlewRateLimiter = new SlewRateLimiter(rateLimitUp, -rateLimitDown, 0);

		this.deadzone = deadzone;
	}

	/** @return Joystick X with the deadzone applied */
	public double getX() {
		return xSlewRateLimiter.calculate(
			xCoefficient * MathUtil.applyDeadband(xSupplier.get(), deadzone)
			* DriverConstants.maxSpeedOptionsTranslation[speedLevel]
		);
	}

	/** @return Joystick Y with the deadzone applied */
	public double getY() {
		return ySlewRateLimiter.calculate(
			yCoefficient * MathUtil.applyDeadband(ySupplier.get(), deadzone)
			* DriverConstants.maxSpeedOptionsTranslation[speedLevel]
		);
	}

	/** @return Joystick rotation with deadzone applied */
	public double getRotation() {
		return rotationSlewRateLimiter.calculate(
			rotationCoefficient * MathUtil.applyDeadband(rotationSupplier.get(), deadzone)
			* DriverConstants.maxSpeedOptionsRotation[speedLevel]
		);
	}

	public void increaseSpeedLevel() {
		speedLevel = Math.min(speedLevel + 1, DriverConstants.NUMBER_OF_SPEED_OPTIONS - 1);
	}

	public void decreaseSpeedLevel() {
		speedLevel = Math.max(speedLevel - 1, 0);
	}

	public void slowMode() {
		speedLevel = 0;
	}
	
	public void normalMode() {
		speedLevel = 1;
	}

	public void fastMode() {
		speedLevel = 2;
	}
	
	public void enableFieldRelative() {
		isFieldRelative = true;
	}

	public void disableFieldRelative() {
		isFieldRelative = false;
	}

	public void toggleFieldRelative() {
		isFieldRelative = !isFieldRelative;
	}

	public boolean isFieldRelative() {
		return isFieldRelative;
	}

	@Override
	public void periodic() {
		SmartDashboard.putString("Speed Mode", DriverConstants.maxSpeedOptionsNames[speedLevel]);
		SmartDashboard.putBoolean("Field Relative", isFieldRelative);
	}
}
