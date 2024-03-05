package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeShooter;

public class SpinFlywheelShooterWithSpinUp extends Command {
	private IntakeShooter intakeShooter;
	private double speed;

	private double spinUpTimeSeconds;

	public SpinFlywheelShooterWithSpinUp(IntakeShooter intakeShooter, double speed, double spinUpTimeSeconds) {
		this.intakeShooter = intakeShooter;
		this.speed = speed;
		this.spinUpTimeSeconds = spinUpTimeSeconds;

		addRequirements(intakeShooter);
	}

	@Override
	public void initialize() {
		intakeShooter.setFlyWheelShooterSpeed(speed);
	}

	@Override
	public boolean isFinished() {
		return intakeShooter.getFlyWheelShooterSpinUpTimeSeconds() > spinUpTimeSeconds;
	}

	@Override
	public void end(boolean interrupted) {
		intakeShooter.setFlyWheelShooterSpeed(0);
	}
}