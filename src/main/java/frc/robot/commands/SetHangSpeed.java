package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hang.Hang;

public class SetHangSpeed extends Command {

	private final Hang hang;
	private final double speed;

	public SetHangSpeed(Hang hang, double speed) {
		this.hang = hang;
		this.speed = speed;
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void initialize() {
		hang.setSpeed(speed);
	}
}
