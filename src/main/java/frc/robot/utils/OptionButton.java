package frc.robot.utils;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Button with different types of control
 */
public class OptionButton {

	private final Supplier<Trigger> button;

	private final ActivationMode mode;

	private boolean isToggled;

	/**
	 * Different modes which a OptionButton can be in
	 */
	public static enum ActivationMode {
		/**
		 * Like toggle, but stays on until manually toggled off with .toggleOff().
		 * Probably just use .onTrue() instead
		 */
		TAP,

		/** standard button behavior, on when pressed, off when released */
		HOLD,

		/** makes button into on/off toggle */
		TOGGLE
	}

	/**
	 * Create Option button.
	 * 
	 * @param controller    device that button is on
	 * @param button        button number
	 * @param mode          whether we want button to act as toggle or hold
	 *                      button
	 */
	public OptionButton(CommandGenericHID controller, int button, ActivationMode mode) {
		this(() -> controller.button(button), mode);
	}

	/**
	 * Create Option button.
	 *
	 * @param button        button number
	 * @param mode          whether we want button to act as toggle or hold button
	 */
	public OptionButton(Supplier<Trigger> button, ActivationMode mode) {
		this.button = button;
		this.mode = mode;

		if (mode == ActivationMode.TOGGLE) {
			button.get().onTrue(new InstantCommand(this::toggle));
		} else if (mode == ActivationMode.TAP) {
			button.get().onTrue(new InstantCommand(this::toggleOn));
		}
	}

	/** Manually toggle state, has no effect if it is hold button */
	public void toggle() {
		isToggled = !isToggled;
	}

	/** Manually toggle state on, has no effect if it is hold button */
	public void toggleOn() {
		isToggled = true;
	}

	/** Manually toggle state off, has no effect if it is hold button */
	public void toggleOff() {
		isToggled = false;
	}

	/**
	 * Get the state of the button
	 * 
	 * <li>for TAP buttons this returns true if the button has been tapped
	 * 
	 * <li>for HOLD buttons this returns true if the button is currently being held down
	 * 
	 * <li> for TOGGLE buttons this returns true if the button has been tapped an odd number of times
	 * 
	 * @return boolean with true being the button is active
	 */
	public boolean getState() {
		switch (mode) {
			case HOLD: return button.get().getAsBoolean();
			case TAP: case TOGGLE: return isToggled;
			default: return false;
		}
	}

	/**
	 * Get state, but return it as a int instead of a boolean
	 * 
	 * @return 1 if true, 0 is false
	 */
	public int getStateAsInt() {
		return getState() ? 1 : 0;
	}
}
