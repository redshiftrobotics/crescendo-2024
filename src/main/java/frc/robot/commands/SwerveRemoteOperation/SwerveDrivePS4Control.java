package frc.robot.commands.SwerveRemoteOperation;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.OptionButton;
import frc.robot.utils.OptionButton.ActivationMode;

/**
 * This is the default command for the drivetrain, allowing for remote operation with PS4 controller
 */
public class SwerveDrivePS4Control extends Command {
    private final SwerveDrivetrain drivetrain;
    private final CommandPS4Controller controller;

    private final OptionButton preciseModeButton;
    private final OptionButton boostModeButton;

    /**
     * Creates a new SwerveDrivePS4Control Command
     * 
     * @param drivetrain The drive train of the robot
     * @param driverPS4Controller The PS4 controller used to control the drive train
     */

    public SwerveDrivePS4Control(SwerveDrivetrain drivetrain, CommandPS4Controller driverPS4Controller){
        this.drivetrain = drivetrain;
        this.controller = driverPS4Controller;

        // Create and configure buttons
        preciseModeButton = new OptionButton(driverPS4Controller,1, ActivationMode.TOGGLE);
        boostModeButton = new OptionButton(driverPS4Controller,2,ActivationMode.HOLD);

        //Tell the command scheduler we are using the drivetrain.
        addRequirements(drivetrain);
    }


    /**
     * The initial subroutine of a command. called once when the command is initially scheduled.
     * Puts all swerve modules to the default state, staying still and facing forwards.
     */
    @Override
    public void initialize(){
        drivetrain.toDefaultStates();
    }


    /**
     * The main body of a command. Called repeatedly while the command is scheduled (Every 20 ms).
     */
    @Override
    public void execute(){
        double leftX = controller.getLeftX();
        double leftY = controller.getLeftY();
        double rightX = controller.getRightX();

        final int speedLevel = 1
            - preciseModeButton.getStateAsInt()
            + boostModeButton.getStateAsInt();

        final ChassisSpeeds speeds = new ChassisSpeeds(
            leftX * DriverConstants.maxSpeedOptionsTranslation[speedLevel],
            leftY * DriverConstants.maxSpeedOptionsTranslation[speedLevel],
            rightX
        );

        drivetrain.setDesiredState(speeds, false);
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end() method and un-schedule it.
     * Always return false since we never want to end in this case.
     */
    @Override
    public boolean isFinished(){
        return false;
    }
    
    /**
     * The action to take when the command ends. Called when either the command finishes normally, or when it interrupted/canceled.
     * Here, this should only happen in this case if we get interrupted.
     */
    @Override
    public void end(boolean interrupted){
        drivetrain.stop();
    }

    // --- Util ---

    /**
     * Utility method. Apply a deadzone to the joystick output to account for stick drift and small bumps.
     * 
     * @param joystickValue Value in [-1, 1] from joystick axis
     * @return {@code 0} if {@code |joystickValue| <= deadzone}, else the {@code joystickValue} scaled to the new control area
     */
    public static double applyJoystickDeadzone(double joystickValue, double deadzone){
        // If the joystick |value| is in the deadzone than zero it out
        if(Math.abs(joystickValue) >= deadzone){
            return 0;
        }

        // scale value from the range [0, 1] to (deadzone, 1]
        return joystickValue * (1 + deadzone) - Math.signum(joystickValue) * deadzone;
    }
}
