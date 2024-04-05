package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// How to make Subsystem: https://compendium.readthedocs.io/en/latest/tasks/subsystems/subsystems.html (ignore image instructions, code is out of date, just look at written general instructions)
// Command based programming: https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
// Subsystem Documentation: https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

public class XboxControllerRumbler extends SubsystemBase {
	/** Constructor. Creates a new ExampleSubsystem. */
	private final XboxController controller;

	private boolean isRumbling = false;

	private final double intensity = 1.0;
	private double stopTime = 0;

	public XboxControllerRumbler(XboxController controller) {
		this.controller = controller;
	}

	/**
	 * This method is called periodically by the CommandScheduler, about every 20ms.
	 * It should be used for updating subsystem-specific state that you don't want
	 * to offload to a Command.
	 * Try to avoid "doing too much" in this method (for example no driver control
	 * here).
	 */
	@Override
	public void periodic() {
		double timeSeconds = (double) System.currentTimeMillis() / 1000.0;

		if (isRumbling) {
			if (timeSeconds >= stopTime) {
				isRumbling = false;
				controller.setRumble(RumbleType.kBothRumble, 0);
			}
		} else {
			if (timeSeconds < stopTime) {
				isRumbling = true;
				controller.setRumble(RumbleType.kBothRumble, intensity);

			}
		}
	}

	public void rumble(double duration) {
		double timeSeconds = (double) System.currentTimeMillis() / 1000.0;
		stopTime = timeSeconds + duration;
	}
}
