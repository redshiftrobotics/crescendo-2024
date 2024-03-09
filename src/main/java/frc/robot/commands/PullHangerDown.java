package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hang.Hang;

public class PullHangerDown extends Command {
	private final Hang hanger;
	private final double speed;

	public PullHangerDown(Hang hanger, double speed) {
		this.hanger = hanger;
		this.speed = speed;
	}

	@Override
	public void initialize() {
		hanger.setSpeed(speed);
	}

	@Override
	public boolean isFinished() {
		return hanger.isAtBottom();
	}

	@Override
	public void end(boolean interrupted) {
		hanger.setSpeed(0);
	}
}
