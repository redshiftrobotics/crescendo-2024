package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightStrip;


public class SetLightstripColor extends Command {
	private LightStrip lightStrip;

	private int r;
	private int g;
	private int b;

	/**
	 * Set the color of the robot's lightstrip
	 * @param lightStrip The lightstrip you want to use (there should only be ONE)
	 * @param r Red level
	 * @param g Green level
	 * @param b Blue level
	 */
	public SetLightstripColor(LightStrip lightStrip, int r, int g, int b) {
		this.lightStrip = lightStrip;
		this.r = r;
		this.g = g;
		this.b = b;
	}

	@Override
	public void initialize() {
		lightStrip.setColor(r, g, b);
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
