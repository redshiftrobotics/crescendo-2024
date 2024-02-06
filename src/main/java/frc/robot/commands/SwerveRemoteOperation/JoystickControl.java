package frc.robot.commands.SwerveRemoteOperation;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.OptionButton;
import frc.robot.utils.OptionButton.ActivationMode;

/**
 * This is the default command for the drivetrain, allowing for remote operation with joystick
 */
public class JoystickControl extends BaseControl {
    private final OptionButton preciseModeButton;
    private final OptionButton boostModeButton;
    private final OptionButton fieldRelativeButton;

    /**
	 * Creates a new SwerveDriveJoystickControl Command.
	 *
	 * @param drivetrain The drivetrain of the robot
	 * @param driverJoystick The joystick used to control drivetrain
	 */
    public JoystickControl(SwerveDrivetrain drivetrain, CommandJoystick driverJoystick) {
        super(drivetrain, driverJoystick);

        // Create and configure buttons
        preciseModeButton = new OptionButton(driverJoystick, 2, ActivationMode.TOGGLE);
        boostModeButton = new OptionButton(driverJoystick, 1, ActivationMode.HOLD);
        fieldRelativeButton = new OptionButton(driverJoystick, 3, ActivationMode.TOGGLE);
    }

    @Override
    public void execute() {
        super.execute();

        final CommandJoystick joystick = (CommandJoystick) controller;

        // Get joystick inputs
        final double speedX = -applyJoystickDeadzone(joystick.getX(), DriverConstants.JOYSTICK_DEAD_ZONE);
		final double speedY = -applyJoystickDeadzone(joystick.getY(), DriverConstants.JOYSTICK_DEAD_ZONE);
        final double speedR = -applyJoystickDeadzone(joystick.getTwist(), DriverConstants.JOYSTICK_DEAD_ZONE);
        
        // Level of speed from Precise, to Normal, to Boost
        // Find our speed level, default is one (Normal)
        final int speedLevel = 1
            - preciseModeButton.getStateAsInt()
            + boostModeButton.getStateAsInt();

        final boolean isFieldRelative = fieldRelativeButton.getState();

        final ChassisSpeeds speeds = new ChassisSpeeds(
            speedY * DriverConstants.maxSpeedOptionsTranslation[speedLevel],
            speedX * DriverConstants.maxSpeedOptionsTranslation[speedLevel],
            speedR * DriverConstants.maxSpeedOptionsRotation[speedLevel]
        );

        // Display relevant data on shuffleboard.
        SmartDashboard.putString("Speed Mode", DriverConstants.maxSpeedOptionsNames[speedLevel]);        
        SmartDashboard.putBoolean("Field Relieve", isFieldRelative);

        drivetrain.setDesiredState(speeds, isFieldRelative, true);
    }
}