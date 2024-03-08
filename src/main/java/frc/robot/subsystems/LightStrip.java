package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants;

public class LightStrip extends SubsystemBase {
	/**
	 * The LED controller PWM signal from 1000-2000us, which is identical to a Spark
	 * servo. So, we use the Spark class.
	 * -1 corresponds to 1000us
	 * 0 corresponds to 1500us
	 * +1 corresponds to 2000us
	 */
	private Spark ledStrip;
	private Spark ledStrip2;
	private double pattern;

	/**
	 * Creates a new LightStrip subsystem.
	 * 
	 * @param pwmPort The port number for the blinkin
	 */
	public LightStrip(int pwmPort) {
		ledStrip = new Spark(pwmPort);
		ledStrip2 = null;
	}

	/**
	 * Creates a new LightStrip subsystem.
	 * 
	 * @param pwmPort The port number for the blinkin
	 */
	public LightStrip(int pwmPort1, int pwmPort2) {
		ledStrip = new Spark(pwmPort1);
		ledStrip2 = new Spark(pwmPort2);
	}

	/**
	 * Sets the blinkin to a pattern
	 * 
	 * @param pattern Pattern ID to use. Consult section 5 of the blinkin manual.
	 * @link https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
	 */
	public void setPattern(double pattern) {
		this.pattern = pattern;
	}

	public void toDefaultPattern() {
		setPattern(LightConstants.LED_COLOR_DEFAULT);
	}

	@Override
	public void periodic() {
		ledStrip.set(pattern);
		if (ledStrip2 != null) {
			ledStrip2.set(pattern);
		}
	}
}
