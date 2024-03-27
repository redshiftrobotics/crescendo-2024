package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hang.Hang;

@SuppressWarnings("unused")
public class PullHangerDown extends Command {
	private final Hang hanger;
	private final double speed;

	public PullHangerDown(Hang hanger, double speed) {
		this.hanger = hanger;
		this.speed = speed;
		
		addRequirements(hanger);
	}

	@Override
	public void initialize() {
		// hanger.setSpeed(speed);
	}

	@Override
	public boolean isFinished() {
		return true;
		// return hanger.isAtBottom();
	}

	@Override
	public void end(boolean interrupted) {
		// hanger.setSpeed(0);
	}
}
