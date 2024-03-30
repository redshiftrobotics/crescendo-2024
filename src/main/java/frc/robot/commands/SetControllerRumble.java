package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class SetControllerRumble extends Command {
	XboxController controller;
	RumbleType rumbleType = RumbleType.kBothRumble;
	double rumbleValue = 0;

	public SetControllerRumble(XboxController controller, double value) {
		this.controller = controller;
		this.rumbleValue = value;
	}

	public SetControllerRumble(XboxController controller, double value, RumbleType rumbleType) {
		this.rumbleType = rumbleType;
		this.controller = controller;
		this.rumbleValue = value;
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
		return true;
	}

	@Override
	public void end(boolean interrupted) {
	}
}
