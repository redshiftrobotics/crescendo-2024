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
        /** Like toggle, but stays on until manfully toggled off. Probably just use .onTrue() instead */
        TAP,

        /** standard button behavior, on when pressed, off when released */
        HOLD,

        /** makes button into on/off toggle */
        TOGGLE
    }
    /**
     * Create Option button.
     * 
     * @param commandDevice device that button is on
     * @param button button number
     * @param mode whether or not we want button to act as toggle or hold button
     */
    public OptionButton(CommandGenericHID controller, int button, ActivationMode mode) {
        this(() -> controller.button(button), mode);
    }
    
    /**
     * Create Option button.
     * 
     * @param commandDevice device that button is on
     * @param button button number
     * @param mode whether or not we want button to act as toggle or hold button
     */
    public OptionButton(Supplier<Trigger> button, ActivationMode mode) {
        this.button = button;
        this.mode = mode;

        if (mode == ActivationMode.TOGGLE) {
            button.get().onTrue(new InstantCommand(this::toggle));
        }
        else if (mode == ActivationMode.TAP) {
            button.get().onTrue(new InstantCommand(this::toggleOn));
        }
    }

    public void toggle() {
        isToggled = !isToggled;
    }

    public void toggleOn() {
        isToggled = true;
    }

    public void toggleOff() {
        isToggled = false;
    }


    /**
     * <p>Get the state of the button</p>
     * 
     * <p>for TAP buttons this returns true if the button has been tapped</p>
     * 
     * <p>for HOLD buttons this returns true if the button is currently being held down</p>
     * 
     * <p>for TOGGLE buttons this returns true if the button has been tapped an odd number of times</p>
     * 
     * @return boolean with true being the button is active
     */
    public boolean getState() {
        switch (mode) {
            case HOLD: return button.get().getAsBoolean();       
            case TAP: return isToggled;
            case TOGGLE: return isToggled;
            default: return false; 
        }
    }

    public int getStateAsInt() {
        return getState() ? 1 : 0;
    }
}
