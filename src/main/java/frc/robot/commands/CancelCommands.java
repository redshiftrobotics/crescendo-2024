package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CancelCommands extends Command {
	public CancelCommands(Subsystem... subsystems) {
		addRequirements(subsystems);
	}

	private static void cancel(Subsystem subsystem) {
		Command current = subsystem.getCurrentCommand();
		if (current != null)
			current.cancel();
	}

	@Override
	public void initialize() {
		getRequirements().forEach(CancelCommands::cancel);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
