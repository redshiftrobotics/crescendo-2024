package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This interface represnets the arm. This interface is implemented by the
 * RealArm (formerly known as Arm) class.
 */
public interface ArmInterface extends Subsystem {
	public void setArmToAmpPosition();

	public void setArmToSpeakerPosition();

	public void setArmToIntakePosition();

	public void setSetpoint(double degree);

	public boolean isAtDesiredPosition();
}
