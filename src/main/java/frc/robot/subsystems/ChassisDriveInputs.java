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

	private final int maxSpeedLevel = DriverConstants.NUMBER_OF_SPEED_OPTIONS - 1;

	private int speedLevel = maxSpeedLevel / 2;
	private boolean isFieldRelative = true;

	private boolean maxSpeedMode = false;

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
			double deadzone, double rateLimitUp, double rateLimitDown) {

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

	public int getSpeedLevel() {
		return maxSpeedMode ? maxSpeedLevel : speedLevel;
	}

	/** @return Joystick X with the deadzone applied */
	public double getX() {
		return xSlewRateLimiter.calculate(
				xCoefficient * MathUtil.applyDeadband(xSupplier.get(), deadzone)
						* DriverConstants.maxSpeedOptionsTranslation[getSpeedLevel()]);
	}

	/** @return Joystick Y with the deadzone applied */
	public double getY() {
		return ySlewRateLimiter.calculate(
				yCoefficient * MathUtil.applyDeadband(ySupplier.get(), deadzone)
						* DriverConstants.maxSpeedOptionsTranslation[getSpeedLevel()]);
	}

	/** @return Joystick rotation with deadzone applied */
	public double getRotation() {
		return rotationSlewRateLimiter.calculate(
				rotationCoefficient * MathUtil.applyDeadband(rotationSupplier.get(), deadzone)
						* DriverConstants.maxSpeedOptionsRotation[getSpeedLevel()]);
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

	public void enableMaxSpeedMode() {
		maxSpeedMode = true;
	}

	public void disableMaxSpeedMode() {
		maxSpeedMode = false;
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
		SmartDashboard.putString("Speed Mode", DriverConstants.maxSpeedOptionsNames[getSpeedLevel()]);
		SmartDashboard.putBoolean("Field Relative", isFieldRelative);
	}
}
