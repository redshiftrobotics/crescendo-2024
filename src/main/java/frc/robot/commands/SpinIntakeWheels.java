package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeShooter;

public class SpinIntakeWheels extends Command {
	private IntakeShooter intakeShooter;
	private double speed;

	public SpinIntakeWheels(IntakeShooter intake, double speed) {
		intakeShooter = intake;
		this.speed = speed;
	}

	@Override
	public void initialize() {
		intakeShooter.setIntakeSpeed(speed);
	}

	// @Override
	// public void execute() { }

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {
		// end
	}
}