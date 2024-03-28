package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeControllerSignal extends Command {
	private final XboxController controller;
	private final BooleanSupplier signal;
	private final double rumbleValue;

	private final double rumbleTimeMax = 0.2 / TimedRobot.kDefaultPeriod;
	private double rumbleTime = 0;

	public IntakeControllerSignal(XboxController controller, BooleanSupplier signal, double rumbleValue) {
		this.controller = controller;
		this.signal = signal;
		this.rumbleValue = rumbleValue;
	}

	@Override
	public void execute() {
		boolean value = signal.getAsBoolean();

		if (value) {
			if (rumbleTime < rumbleTimeMax) {
				rumbleTime++;
				controller.setRumble(RumbleType.kBothRumble, rumbleValue);
			}
		}
		else {
			rumbleTime = 0;
			controller.setRumble(RumbleType.kBothRumble, 0);
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		controller.setRumble(RumbleType.kBothRumble, 0);
	}
}
