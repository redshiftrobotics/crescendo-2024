package frc.robot.commands.SwerveRemoteOperation;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.OptionButton;
import frc.robot.utils.OptionButton.ActivationMode;

// Here is the documentation for the xbox controller code:
// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/CommandXboxController.html

/**
 * This is the default command for the drivetrain, allowing for remote operation
 * with xbox controller
 */
public class SwerveDriveXboxControl extends SwerveDriveBaseControl {
	private final OptionButton preciseModeButton;
	private final OptionButton boostModeButton;
	private final OptionButton fieldRelativeButton;

	// private final OptionButton fieldRelativeButton;

	/**
	 * Creates a new SwerveDriveXboxControl Command.
	 *
	 * @param drivetrain           The drivetrain of the robot
	 * @param driverXboxController The xbox controller used to control drivetrain
	 */
	public SwerveDriveXboxControl(SwerveDrivetrain drivetrain, CommandXboxController driverXboxController) {
		super(drivetrain, driverXboxController);

		// Create and configure buttons
		// OptionButton exampleToggleButton = new OptionButton(controller::a,
		// ActivationMode.TOGGLE);
		preciseModeButton = new OptionButton(driverXboxController::b, ActivationMode.TOGGLE);
		boostModeButton = new OptionButton(driverXboxController::leftStick, ActivationMode.HOLD);
		fieldRelativeButton = new OptionButton(driverXboxController::povUp, ActivationMode.TOGGLE);

		// fieldRelativeButton = new OptionButton(driverXboxController::,
		// ActivationMode.TOGGLE)

		// Tell the command schedular we are using the drivetrain
		addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		final CommandXboxController xboxController = (CommandXboxController) controller;

		final double leftX = applyJoystickDeadzone(xboxController.getLeftX(), DriverConstants.XBOX_DEAD_ZONE);
		final double leftY = applyJoystickDeadzone(xboxController.getLeftY(), DriverConstants.XBOX_DEAD_ZONE);

		final double rightX = -applyJoystickDeadzone(xboxController.getRightX(), DriverConstants.XBOX_DEAD_ZONE);

		final boolean isFieldRelative = fieldRelativeButton.getState();

		final int speedLevel = 1
				- preciseModeButton.getStateAsInt()
				+ boostModeButton.getStateAsInt();

		final ChassisSpeeds speeds = new ChassisSpeeds(
				leftX * DriverConstants.maxSpeedOptionsTranslation[speedLevel],
				leftY * DriverConstants.maxSpeedOptionsTranslation[speedLevel],
				rightX * DriverConstants.maxSpeedOptionsRotation[speedLevel]);

		// Display relevant data on shuffleboard.
		SmartDashboard.putString("Speed Mode", DriverConstants.maxSpeedOptionsNames[speedLevel]);
		SmartDashboard.putBoolean("Field Relieve", isFieldRelative);

		drivetrain.setDesiredState(speeds, true, isFieldRelative);
	}
}