package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmInterface;

public class ArmRotateTo extends Command {

	private final double setpoint;
	private final ArmInterface arm;

	public ArmRotateTo(ArmInterface arm, double degree) {
		this.setpoint = degree;
		this.arm = arm;

		addRequirements(arm);

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
