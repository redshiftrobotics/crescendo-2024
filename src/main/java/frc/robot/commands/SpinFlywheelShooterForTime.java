package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeShooter;

public class SpinFlywheelShooterForTime extends Command {
	private final IntakeShooter intakeShooter;
	private final double speed;
	private final double millisToWait;

	public SpinFlywheelShooterForTime(IntakeShooter intakeShooter, double speed, double seconds) {
		this.intakeShooter = intakeShooter;
		this.speed = speed;
		this.millisToWait = Units.secondsToMilliseconds(seconds);

		addRequirements(intakeShooter);
	}

	@Override
	public void initialize() {
		intakeShooter.setFlyWheelShooterSpeed(speed);
	}

	@Override
	public boolean isFinished() {
		return intakeShooter.howLongAtSpeedMillis(speed) > millisToWait;
	}
}