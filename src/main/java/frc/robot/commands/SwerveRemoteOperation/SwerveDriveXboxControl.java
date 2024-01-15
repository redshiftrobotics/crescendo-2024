package frc.robot.commands.SwerveRemoteOperation;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveDrivetrain;

/**
 * This is the default command for the drivetrain, allowing for remote operation with joystick
 */
public class SwerveDriveXboxControl extends SwerveDriveControl<CommandXboxController> {
    /**
	 * Creates a new SwerveDriveXboxControl Command.
	 *
	 * @param drivetrain The drivetrain of the robot
	 * @param driverXboxController The xbox controller used to control drivetrain
	 */
    public SwerveDriveXboxControl(SwerveDrivetrain drivetrain, CommandXboxController driverXboxController) {
        super(drivetrain, driverXboxController);

        // Create and configure buttons
        // OptionButton exampleToggleButton = new OptionButton(controller::a, ActivationMode.TOGGLE);

        // Tell the command schedular we are using the drivetrain
        addRequirements(drivetrain);
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled (Every 20 ms).
     */
    @Override
    public void execute() {

    }
}