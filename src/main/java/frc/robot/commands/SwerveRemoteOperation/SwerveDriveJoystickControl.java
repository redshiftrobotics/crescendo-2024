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
public class SwerveDriveJoystickControl extends SwerveDriveControl<CommandJoystick> {
    private OptionButton preciseModeButton;
    private OptionButton boostModeButton;
    private OptionButton fieldRelieveButton;

    /**
	 * Creates a new SwerveDriveJoystickControl Command.
	 *
	 * @param drivetrain The drivetrain of the robot
	 * @param driverJoystick The joystick used to control drivetrain
	 */
    public SwerveDriveJoystickControl(SwerveDrivetrain drivetrain, CommandJoystick driverJoystick) {
        super(drivetrain, driverJoystick);

        // Create and configure buttons
        preciseModeButton = new OptionButton(driverJoystick, 2, ActivationMode.TOGGLE);
        boostModeButton = new OptionButton(driverJoystick, 1, ActivationMode.HOLD);
        fieldRelieveButton = new OptionButton(driverJoystick, 3, ActivationMode.TOGGLE);

        // Tell the command schedular we are using the drivetrain
        addRequirements(drivetrain);
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled (Every 20 ms).
     */
    @Override
    public void execute() {

        // Get joystick inputs
        final double speedX = applyJoystickDeadzone(-controller.getX(), DriverConstants.JOYSTICK_DEAD_ZONE);
		final double speedY = applyJoystickDeadzone(-controller.getY(), DriverConstants.JOYSTICK_DEAD_ZONE);
		final double speedOmega = applyJoystickDeadzone(controller.getTwist(), DriverConstants.JOYSTICK_DEAD_ZONE);

        // // Code for rotating with buttons if driver prefers 
        // double speedOmega = 0;
        // if (joystick.button(7).getAsBoolean()) {
		// 	speedOmega += OperatorConstants.maxSpeedOptionsRotation[0];
		// } else if (joystick.button(8).getAsBoolean()) {
		// 	speedOmega -= OperatorConstants.maxSpeedOptionsRotation[0];
		// } else if (joystick.button(9).getAsBoolean()) {
		// 	speedOmega += OperatorConstants.maxSpeedOptionsRotation[1];
		// } else if (joystick.button(10).getAsBoolean()) {
		// 	speedOmega -= OperatorConstants.maxSpeedOptionsRotation[1];
		// }

        
        // Level of speed from Precise, to Normal, to Boost
        // Find our speed level, default is one (Normal)
        final int speedLevel = 1
            - preciseModeButton.getStateAsInt()
            + boostModeButton.getStateAsInt();

        // Can be changed for testing
        final int speedCoefficient = 1;

        final boolean isFieldRelative = fieldRelieveButton.getState();

        final ChassisSpeeds speeds = new ChassisSpeeds(
            speedX * DriverConstants.maxSpeedOptionsTranslation[speedLevel] * speedCoefficient,
            speedY * DriverConstants.maxSpeedOptionsTranslation[speedLevel] * speedCoefficient,
            speedOmega * DriverConstants.maxSpeedOptionsRotation[speedLevel] * speedCoefficient
        );

        // Display relevant data on shuffleboard.
        SmartDashboard.putString("Speed Mode", DriverConstants.maxSpeedOptionsNames[speedLevel]);        
        SmartDashboard.putBoolean("Field Relieve", isFieldRelative);

        SmartDashboard.putNumber("Heading Degrees", drivetrain.getHeading().getDegrees());

        // (60 / 2*Math.PI) = 94.24777960769379
        final double radiansPerSecondToRPM = 94.24777960769379;
        final double metersPerSecondToMPH = 2.2369;

        SmartDashboard.putNumber("Target Speed X MPH", speeds.vxMetersPerSecond * metersPerSecondToMPH);
        SmartDashboard.putNumber("Target Speed Y MPH", speeds.vyMetersPerSecond * metersPerSecondToMPH);
        SmartDashboard.putNumber("Target RPM", speeds.omegaRadiansPerSecond * radiansPerSecondToRPM);

        final ChassisSpeeds realSpeeds = drivetrain.getState();

        SmartDashboard.putNumber("Real Speed X MPH", realSpeeds.vxMetersPerSecond * metersPerSecondToMPH);
        SmartDashboard.putNumber("Real Speed Y MPH", realSpeeds.vyMetersPerSecond * metersPerSecondToMPH);
        SmartDashboard.putNumber("Real RPM", realSpeeds.omegaRadiansPerSecond * radiansPerSecondToRPM);

        SmartDashboard.putNumber("Joystick X", controller.getX());
		SmartDashboard.putNumber("Joystick Y", controller.getY());
		SmartDashboard.putNumber("Joystick R", controller.getTwist());

        SmartDashboard.putBoolean("X Active", speedX != 0);
		SmartDashboard.putBoolean("Y Active", speedY != 0);
		SmartDashboard.putBoolean("R Active", speedOmega != 0);


        drivetrain.setDesiredState(speeds, isFieldRelative);
    }
}