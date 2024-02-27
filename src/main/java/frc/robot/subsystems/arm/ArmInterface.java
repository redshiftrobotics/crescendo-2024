package frc.robot.subsystems.arm;

/**
 * This interface represnets the arm. This interface is implemented by the
 * RealArm (formerly known as Arm) class.
 */
public interface ArmInterface {
	public void changeArmAngleDegreesBy(double desiredDegrees);

	public void setArmToAmpPosition();

	public void setArmToSpeakerPosition();

	public void setArmToIntakePosition();

	public void setSetpoint(double degree);

	public boolean isAtDesiredPosition();
}
