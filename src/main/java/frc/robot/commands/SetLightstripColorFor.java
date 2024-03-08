package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightStrip;

public class SetLightstripColorFor extends Command {
	private final LightStrip lightStrip;
	private final double pattern;

	private final String name;

	private double secondsLeftMillis;

	private double delayTimeMillis;
	private double endTimeMillis;

	/**
	 * Set the color of the robot's lightstrip for a certain number of seconds
	 * 
	 * @param lightStrip The lightstrip you want to use (there should only be ONE)
	 * @param pattern    Pattern
	 * @param seconds    Seconds to keep it that color
	 * @param name		 SmartDashboard name
	 */
	public SetLightstripColorFor(LightStrip lightStrip, double pattern, double seconds, String name) {
		this.lightStrip = lightStrip;
		this.pattern = pattern;

		this.name = name;
		this.delayTimeMillis = Units.secondsToMilliseconds(seconds);

		addRequirements(lightStrip);
	}

	public SetLightstripColorFor(LightStrip lightStrip, double pattern, double seconds) {
		this(lightStrip, pattern, seconds, "p" + String.valueOf(pattern));
	}

	@Override
	public void initialize() {
		lightStrip.setPattern(pattern);
		this.endTimeMillis = System.currentTimeMillis() + delayTimeMillis;
	}

	@Override
	public void execute() {
		secondsLeftMillis = endTimeMillis - System.currentTimeMillis();
		SmartDashboard.putString("LED Signal", (name + " " + Math.round(Units.millisecondsToSeconds(secondsLeftMillis))));
	}

	@Override
	public boolean isFinished() {
		return secondsLeftMillis < 0;
	}

	@Override
	public void end(boolean interrupted) {
		lightStrip.toDefaultPattern();
		SmartDashboard.putString("LED Signal", "Default");
	}
}
