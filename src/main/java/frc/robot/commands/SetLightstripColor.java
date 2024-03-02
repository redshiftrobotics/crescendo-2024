package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightStrip;

public class SetLightstripColor extends Command {
	private LightStrip lightStrip;

	private double pattern;

	/**
	 * Set the color of the robot's lightstrip
	 * 
	 * @param lightStrip The lightstrip you want to use (there should only be ONE)
	 * @param pattern    Pattern
	 */
	public SetLightstripColor(LightStrip lightStrip, double pattern) {
		this.lightStrip = lightStrip;
		this.pattern = pattern;
	}

	@Override
	public void initialize() {
		System.out.println("*** Command: Selecting pattern " + pattern);
		lightStrip.setPattern(pattern);
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {
		// Don't think I have to do anything
	}
}