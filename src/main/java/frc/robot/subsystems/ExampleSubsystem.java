package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// How to make Subsystem: https://compendium.readthedocs.io/en/latest/tasks/subsystems/subsystems.html (ignore image instructions, code is out of date, just look at written general instructions)
// Command based programming: https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
// Subsystem Documentation documentation: https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html

/**
 * How to make a Subsystem:
 * 1. Copy this file, remember that class name has to match 
 */

public class ExampleSubsystem extends SubsystemBase {
    /** Constructor. Creates a new ExampleSubsystem. */
    public ExampleSubsystem() {

    }

    /**
     * This method is called periodically by the CommandScheduler, about every 20ms.
     * It should be used for updating subsystem-specific state that you don't want to offload to a Command.
     * Try to avoid "doing to much" in this method (for example no driver control here).
     */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
