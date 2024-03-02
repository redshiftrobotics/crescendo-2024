package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeShooter;

public class SpinIntakeFlywheels extends Command {
	private IntakeShooter intakeShooter;
	private int speed;

	private long startTime;

	public SpinIntakeFlywheels(IntakeShooter intake, int speed) {
		intakeShooter = intake;
		this.speed = speed;
	}

	@Override
	public void initialize() {
		startTime = System.currentTimeMillis();
		intakeShooter.setFlyWheelSpeed(speed);
	}

	// @Override
	// public void execute() { }

	@Override
	public boolean isFinished() {
		return (System.currentTimeMillis() + 1000) >= startTime;
	}

	@Override
	public void end(boolean interrupted) {
		// end
	}
}