package frc.robot.commands.SwerveRemoteOperation;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.OptionButton;
import frc.robot.utils.OptionButton.ActivationMode;

/**
 * This is the default command for the drivetrain, allowing for remote operation with joystick
 */
public class SwerveDriveJoystickControl extends Command {
    private final SwerveDrivetrain drivetrain;
    private final CommandJoystick joystick;

    private final OptionButton preciseModeButton;
    private final OptionButton boostModeButton;
    private final OptionButton fieldRelativeButton;

    /**
	 * Creates a new SwerveDriveJoystickControl Command.
	 *
	 * @param drivetrain The drivetrain of the robot
	 * @param driverJoystick The joystick used to control drivetrain
	 */
    public SwerveDriveJoystickControl(SwerveDrivetrain drivetrain, CommandJoystick driverJoystick) {
        this.drivetrain = drivetrain;
        this.joystick = driverJoystick;

        // Create and configure buttons
        preciseModeButton = new OptionButton(driverJoystick, 2, ActivationMode.TOGGLE);
        boostModeButton = new OptionButton(driverJoystick, 1, ActivationMode.HOLD);
        fieldRelativeButton = new OptionButton(driverJoystick, 3, ActivationMode.TOGGLE);

        // Tell the command schedular we are using the drivetrain
        addRequirements(drivetrain);
    }

    
    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     * Puts all swerve modules to the default state, staying still and facing forwards.
     */
    @Override
    public void initialize() {
        drivetrain.toDefaultStates();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled (Every 20 ms).
     */
    @Override
    public void execute() {

        // Get joystick inputs
        final double speedX = -applyJoystickDeadzone(joystick.getX(), DriverConstants.JOYSTICK_DEAD_ZONE);
		final double speedY = -applyJoystickDeadzone(joystick.getY(), DriverConstants.JOYSTICK_DEAD_ZONE);
		
        double speedR = -applyJoystickDeadzone(joystick.getTwist(), DriverConstants.JOYSTICK_DEAD_ZONE);

        // Code for rotating with buttons if driver prefers 
		if (joystick.button(9).getAsBoolean()) {
			speedR += DriverConstants.maxSpeedOptionsRotation[1];
        }
		if (joystick.button(10).getAsBoolean()) {
			speedR -= DriverConstants.maxSpeedOptionsRotation[1];
		}

        
        // Level of speed from Precise, to Normal, to Boost
        // Find our speed level, default is one (Normal)
        final int speedLevel = 1
            - preciseModeButton.getStateAsInt()
            + boostModeButton.getStateAsInt();

        // Can be changed for testing
        final int speedCoefficient = 1;

        final boolean isFieldRelative = fieldRelativeButton.getState();

        final ChassisSpeeds speeds = new ChassisSpeeds(
            speedY * DriverConstants.maxSpeedOptionsTranslation[speedLevel] * speedCoefficient,
            speedX * DriverConstants.maxSpeedOptionsTranslation[speedLevel] * speedCoefficient,
            speedR * DriverConstants.maxSpeedOptionsRotation[speedLevel] * speedCoefficient
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

        final ChassisSpeeds currentRobotSpeeds = drivetrain.getState();

        SmartDashboard.putNumber("Real Speed X MPH", currentRobotSpeeds.vxMetersPerSecond * metersPerSecondToMPH);
        SmartDashboard.putNumber("Real Speed Y MPH", currentRobotSpeeds.vyMetersPerSecond * metersPerSecondToMPH);
        SmartDashboard.putNumber("Real RPM", currentRobotSpeeds.omegaRadiansPerSecond * radiansPerSecondToRPM);

        SmartDashboard.putNumber("Joystick X", joystick.getX());
		SmartDashboard.putNumber("Joystick Y", joystick.getY());
		SmartDashboard.putNumber("Joystick R", joystick.getTwist());

        SmartDashboard.putBoolean("X Active", speedX != 0);
		SmartDashboard.putBoolean("Y Active", speedY != 0);
		SmartDashboard.putBoolean("R Active", speedR != 0); 

        drivetrain.setDesiredState(speeds, isFieldRelative);
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end() method and un-schedule it.
     * Always return false since we never want to end in this case.
     */
	@Override
	public boolean isFinished() {
		return false;
	}

    /**
     * The action to take when the command ends. Called when either the command finishes normally, or when it interrupted/canceled.
     * Here, this should only happen in this case if we get interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    /* -- Util --- */


    /**
     * Utility method. Apply a deadzone to the joystick output to account for stick drift and small bumps.
     * 
     * @param joystickValue Value in [-1, 1] from joystick axis
     * @return {@code 0} if {@code |joystickValue| <= deadzone}, else the {@code joystickValue} scaled to the new control area
     */
    public static double applyJoystickDeadzone(double joystickValue, double deadzone) {
        if (Math.abs(joystickValue) <= deadzone) {
            // If the joystick |value| is in the deadzone than zero it out
            return 0;
        }

        // scale value from the range [0, 1] to (deadzone, 1]
        return joystickValue * (1 + deadzone) - Math.signum(joystickValue) * deadzone;
    }
}