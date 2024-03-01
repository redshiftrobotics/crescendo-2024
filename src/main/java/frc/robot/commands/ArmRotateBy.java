package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmRotateBy extends Command {

	private final Rotation2d dSetpoint;
	private final Arm arm;

	public ArmRotateBy(Arm arm, Rotation2d dRotation) {
		this.dSetpoint = dRotation;
		this.arm = arm;

		addRequirements(arm);
	}

	public ArmRotateBy(Arm arm, double dDegrees) {
		this(arm, Rotation2d.fromDegrees(dDegrees));
	}

	@Override
	public void initialize() {
		arm.setSetpoint(arm.getArmPosition().plus(dSetpoint));
	}

	@Override
	public boolean isFinished() {
		// return arm.isAtDesiredPosition();
		return true;
	}
}
