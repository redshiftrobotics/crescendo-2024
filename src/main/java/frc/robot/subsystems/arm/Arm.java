package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

/**
 * This interface represnets the arm. This interface is implemented by the
 * RealArm (formerly known as Arm) class.
 */
public abstract class Arm extends SubsystemBase {

	public abstract void setSetpoint(double degrees);
	public abstract void setSetpoint(Rotation2d rotation);

	public abstract boolean isAtDesiredPosition();

	public abstract Rotation2d getArmPosition();

	public void setArmToAmpPosition() {
		setSetpoint(ArmConstants.ARM_AMP_SHOOTING_DEGREES);
	}

	public void setArmToSpeakerPosition() {
		setSetpoint(ArmConstants.ARM_SPEAKER_FRONT_SHOOTING_DEGREES);
	}

	public void setArmToIntakePosition() {
		setSetpoint(ArmConstants.ARM_INTAKE_DEGREES);
	}

	public void setArmToStartPosition() {
		setSetpoint(ArmConstants.ARM_STOW_2_DEGREES);
	}

	public void setArmToDrivePosition() {
		setSetpoint(ArmConstants.ARM_STOW_DEGREES);
	}
}
