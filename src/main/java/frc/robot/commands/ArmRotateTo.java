package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmRotateTo extends Command {

	private final Rotation2d setpoint;
	private final Arm arm;

	public ArmRotateTo(Arm arm, Rotation2d rotation) {
		this.setpoint = rotation;
		this.arm = arm;
		
		addRequirements(arm);
	}

	public ArmRotateTo(Arm arm, double degrees) {
		this(arm, Rotation2d.fromDegrees(degrees));
	}

	@Override
	public void initialize() {
		arm.setSetpoint(setpoint);
	}

	@Override
	public boolean isFinished() {
		return arm.isAtDesiredPosition();
	}
}
