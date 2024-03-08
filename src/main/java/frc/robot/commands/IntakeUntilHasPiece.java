package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeShooter;

public class IntakeUntilHasPiece extends Command {
	private final IntakeShooter intakeShooter;
	private final double speed;

	public IntakeUntilHasPiece(IntakeShooter intakeShooter, double speed) {
		this.intakeShooter = intakeShooter;
		this.speed = speed;

		addRequirements(intakeShooter);
	}

	@Override
	public void initialize() {
		intakeShooter.setIntakeGrabberSpeed(speed);
	}

	@Override
	public boolean isFinished() {
		return intakeShooter.hasNoteInIntake();
	}

	@Override
	public void end(boolean interrupted) {
		intakeShooter.setIntakeGrabberSpeed(0);
	}
}