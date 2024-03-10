package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hang.Hang;

public class HangControl extends Command {
	private final Hang hanger;
	private final Supplier<Double> speedSupplier;

	private final double MILLIS_TILL_BLOCK = Units.secondsToMilliseconds(0.15);
	private Double timeToBeginBlock;

	public HangControl(Hang hanger, Supplier<Double> speedSupplier) {
		this.hanger = hanger;
		this.speedSupplier = speedSupplier;
		addRequirements(hanger);
	}

	@Override
	public void initialize() {
		timeToBeginBlock = null;
	}

	@Override
	public void execute() {

		boolean useMagneticLimitSwitches = false;

		double speed = speedSupplier.get();

		if (useMagneticLimitSwitches) {
			double currentTime = System.currentTimeMillis();
	
			if (!hanger.isAtBottom()) {
				timeToBeginBlock = null;
			} else if (timeToBeginBlock == null) {
				timeToBeginBlock = currentTime + MILLIS_TILL_BLOCK;
			}
	
			if (timeToBeginBlock != null && currentTime >= timeToBeginBlock) {
				speed = 0;
			}
		}

		hanger.setSpeed(speed);
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		hanger.setSpeed(0);
	}
}
