package frc.robot.inputs;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriverConstants;

/** Class that stores supplies for main controls of ChassisSpeeds */
public class ChassisDriveInputs {

	private final Supplier<Double> xSupplier, ySupplier, rotationSupplier;

	private final double xCoefficient, yCoefficient, rotationCoefficient;

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
			double deadzone) {

		this.ySupplier = getForward;
		this.xSupplier = getLeft;
		this.rotationSupplier = getRotation;

		this.yCoefficient = forwardCoefficient;
		this.xCoefficient = leftCoefficient;
		this.rotationCoefficient = rotationCoefficient;

		this.deadzone = deadzone;

		SmartDashboard.putString("Speed Mode", getSpeedLevelName());
	}

	/** @return Joystick X with the deadzone applied */
	public double getX() {
		return applyJoystickDeadzone(xSupplier.get(), deadzone) * xCoefficient
				* DriverConstants.maxSpeedOptionsTranslation[speedLevel];
	}

	/** @return Joystick Y with the deadzone applied */
	public double getY() {
		return applyJoystickDeadzone(ySupplier.get(), deadzone) * yCoefficient
				* DriverConstants.maxSpeedOptionsTranslation[speedLevel];
	}

	/** @return Joystick rotation with deadzone applied */
	public double getRotation() {
		return applyJoystickDeadzone(rotationSupplier.get(), deadzone) * rotationCoefficient
				* DriverConstants.maxSpeedOptionsRotation[speedLevel];
	}

	public void increaseSpeedLevel() {
		speedLevel = Math.min(speedLevel + 1, DriverConstants.NUMBER_OF_SPEED_OPTIONS);
		SmartDashboard.putString("Speed Mode", getSpeedLevelName());
	}

	public void decreaseSpeedLevel() {
		speedLevel = Math.max(speedLevel - 1, 0);
		SmartDashboard.putString("Speed Mode", getSpeedLevelName());
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

	public String getSpeedLevelName() {
		return DriverConstants.maxSpeedOptionsNames[speedLevel];
	}

	/**
	 * Utility method. Apply a deadzone to the joystick output to account for stick
	 * drift and small bumps.
	 * 
	 * @param joystickValue Value in [-1, 1] from joystick axis
	 * @return {@code 0} if {@code |joystickValue| <= deadzone}, else the
	 *         {@code joystickValue} scaled to the new control area
	 */
	private static double applyJoystickDeadzone(double joystickValue, double deadzone) {
		if (Math.abs(joystickValue) <= deadzone) {
			// If the joystick |value| is in the deadzone than zero it out
			return 0;
		}

		// scale value from the range [0, 1] to (deadzone, 1]
		return joystickValue * (1 + deadzone) - Math.signum(joystickValue) * deadzone;
	}
}
