package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightStrip;

public class IntakeLEDSignal extends Command {
	private final LightStrip LEDs;
	private final BooleanSupplier signal;
	private final double pattern;

	public IntakeLEDSignal(LightStrip LEDs, BooleanSupplier signal, double pattern) {
		this.LEDs = LEDs;
		this.signal = signal;
		this.pattern = pattern;
	}

	@Override
	public void execute() {
		if (signal.getAsBoolean()) {
			LEDs.setPattern(pattern);
		}
		else {
			LEDs.off();
		}
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		LEDs.off();
	}
}
