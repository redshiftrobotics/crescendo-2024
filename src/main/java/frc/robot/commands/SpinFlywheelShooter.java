package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeShooter;

public class SpinFlywheelShooter extends Command {
	private IntakeShooter intakeShooter;
	private double speed;

	public SpinFlywheelShooter(IntakeShooter intakeShooter, double speed) {
		this.intakeShooter = intakeShooter;
		this.speed = speed;

		addRequirements(intakeShooter);
	}

	@Override
	public void initialize() {
		intakeShooter.setFlyWheelShooterSpeed(speed);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}