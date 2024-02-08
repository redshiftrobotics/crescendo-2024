package frc.robot.utils;

import java.util.function.Supplier;

public class ChassisDriveInputs {

	private final Supplier<Double> xSupplier;
	private final Supplier<Double> ySupplier;
	private final Supplier<Double> rotationSupplier;

	private final double deadzone;

	private final double translationCoefficient;
	private final double rotationCoefficient;

	public ChassisDriveInputs(
			Supplier<Double> getX, Supplier<Double> getY, Supplier<Double> getRotation,
			double translationCoefficient, double rotationCoefficient, double deadzone) {

		this.xSupplier = getX;
		this.ySupplier = getY;
		this.rotationSupplier = getRotation;

		this.deadzone = deadzone;

		this.translationCoefficient = translationCoefficient;
		this.rotationCoefficient = rotationCoefficient;
	}

	public double getX() {
		return applyJoystickDeadzone(xSupplier.get(), deadzone) * translationCoefficient;
	}

	public double getY() {
		return applyJoystickDeadzone(ySupplier.get(), deadzone) * translationCoefficient;
	}

	public double getRotation() {
		return applyJoystickDeadzone(rotationSupplier.get(), deadzone) * rotationCoefficient;
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
