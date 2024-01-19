package frc.robot.commands.SwerveRemoteOperation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.SwerveDrivetrain;

// Here is the documentation for the xbox controller code:
// https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/CommandXboxController.html

/**
 * This is the default command for the drivetrain, allowing for remote operation with xbox controller
 */
public class SwerveDriveXboxControl extends Command {
    private final SwerveDrivetrain drivetrain;
    private final CommandXboxController controller;
    private final PIDController robotAnglePID;

    /**
     * Creates a new SwerveDriveXboxControl Command.
     *
     * @param drivetrain           The drivetrain of the robot
     * @param driverXboxController The xbox controller used to control drivetrain
     */
    public SwerveDriveXboxControl(SwerveDrivetrain drivetrain, CommandXboxController driverXboxController) {
        this.drivetrain = drivetrain;
        this.controller = driverXboxController;

        robotAnglePID = new PIDController(
                ControllerConstants.CONTROLLER_PID_P,
                ControllerConstants.CONTROLLER_PID_I,
                ControllerConstants.CONTROLLER_PID_D);
        robotAnglePID.enableContinuousInput(0, 2 * Math.PI);

        // Create and configure buttons
        // OptionButton exampleToggleButton = new OptionButton(controller::a, ActivationMode.TOGGLE);

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

        double leftX = applyJoystickDeadzone(controller.getLeftX(), DriverConstants.XBOX_DEAD_ZONE);
        double leftY = applyJoystickDeadzone(controller.getLeftY(), DriverConstants.XBOX_DEAD_ZONE);

        double rightX = applyJoystickDeadzone(controller.getRightX(), DriverConstants.XBOX_DEAD_ZONE);
        double rightY = applyJoystickDeadzone(controller.getRightY(), DriverConstants.XBOX_DEAD_ZONE);

        double targetAngle = Math.atan2(rightX, rightY);

        final double currentAngle = drivetrain.getHeading().getRadians();
        final double turnSpeed = robotAnglePID.calculate(currentAngle, targetAngle);

        final ChassisSpeeds speeds = new ChassisSpeeds(
                leftX * ControllerConstants.maxSpeed,
                leftY * ControllerConstants.maxSpeed,
                turnSpeed);

        drivetrain.setDesiredState(speeds);

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

    // --- Util ---

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