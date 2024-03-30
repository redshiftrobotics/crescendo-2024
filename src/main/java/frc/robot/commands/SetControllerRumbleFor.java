package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class SetControllerRumbleFor extends Command {
	XboxController controller;
	RumbleType rumbleType = RumbleType.kBothRumble;
	double rumbleValue = 0;
	long startTime;
	long endTime;

	public SetControllerRumbleFor(XboxController controller, double value, long durationSec) {
		this.controller = controller;
		this.rumbleValue = value;

		this.startTime = System.currentTimeMillis();
		this.endTime = startTime + durationSec;
	}

	public SetControllerRumbleFor(XboxController controller, double value, long durationSec, RumbleType rumbleType) {
		this.rumbleType = rumbleType;
		this.controller = controller;
		this.rumbleValue = value;

		this.startTime = System.currentTimeMillis();
		this.endTime = startTime + durationSec;
	}

	@Override
	public void initialize() {
		controller.setRumble(rumbleType, rumbleValue);
	}

	@Override
	public void execute() {
	}

	@Override
	public boolean isFinished() {
		return endTime >= System.currentTimeMillis();
	}

	@Override
	public void end(boolean interrupted) {
		controller.setRumble(rumbleType, 0);
	}
}
