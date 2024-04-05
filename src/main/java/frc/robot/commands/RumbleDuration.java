package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XboxControllerRumbler;

// How to make Command: https://compendium.readthedocs.io/en/latest/tasks/commands/commands.html (ignore image instructions, code is out of date, just look at written general instructions)
// Command based programming: https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
// Code documentations https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html 

/** An example command that uses an example subsystem. */
public class RumbleDuration extends Command {
	private final XboxControllerRumbler rumbler;
	private final double duration;

	/**
	 * The constructor creates a new command and is automatically called one time
	 * when the command is created (with 'new' keyword).
	 * It should set up the initial state and properties of the object to ensure
	 * it's ready for use.
	 * This can take in any arguments you need. It normally uses 1 subsystem (but an
	 * take multiple when necessary),
	 * as wells as arguments for what to do, such as a joystick in the drive command
	 * or a desired position in an auto command.
	 * Example uses include saving parameters passed to the command, creating and
	 * configuring objects for the class like PID controllers, and adding subsystem
	 * requirements
	 */
	public RumbleDuration(XboxControllerRumbler rumbler, double duration) {
		this.rumbler = rumbler;
		this.duration = duration;

		addRequirements(rumbler);
	}

	/**
	 * initialize() is used to prepare a command for execution and is called once
	 * when the command is scheduled.
	 * It should reset the command's state since command objects can be used
	 * multiple times.
	 * Example uses include setting motor to constant speed, setting a solenoid to a
	 * certain state, and resetting variables
	 */
	@Override
	public void initialize() {
		this.rumbler.rumble(duration);
	}

	/**
	 * isFinished() finished is called repeatedly while a command is scheduled,
	 * right after execute.
	 * It should return true when you want the command to finish. end(false) is
	 * called directly after isFinished() returns true.
	 * Example uses include checking if control loop is at set point, and always
	 * returning false to end after just 1 call to execute.
	 */
	@Override
	public boolean isFinished() {
		return true;
	}
}
