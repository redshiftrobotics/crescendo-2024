package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This is a dummy stand in class that represents the arm, in the event that the
 * robot does not have an arm in order to prevent errors.
 */
public class DummyArm extends SubsystemBase implements ArmInterface {
	public void setArmToAmpPosition() {
	}

	public void setArmToSpeakerPosition() {
	}

	public void setArmToIntakePosition() {
	}

	public void setSetpoint(double degree) {
		System.out.println(degree);
	}

	public boolean isAtDesiredPosition() {
		return true;
	}
}