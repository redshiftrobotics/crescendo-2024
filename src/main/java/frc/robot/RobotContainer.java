package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.SwerveRemoteOperation.SwerveDriveJoystickControl;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Vision;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    // private final SwerveModule swerveModuleFL = new SwerveModule(
    //         SwerveModuleConstants.VELOCITY_MOTOR_ID_FL,
	// 		SwerveModuleConstants.ANGULAR_MOTOR_ID_FL,
	// 		SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FL,
	// 		SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FL,
    //         new Translation2d(SwerveDrivetrainConstants.MODULE_LOCATION_X, SwerveDrivetrainConstants.MODULE_LOCATION_Y));
	// private final SwerveModule swerveModuleFR = new SwerveModule(
    //         SwerveModuleConstants.VELOCITY_MOTOR_ID_FR,
	// 		SwerveModuleConstants.ANGULAR_MOTOR_ID_FR,
	// 		SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FR,
	// 		SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FR,
    //         new Translation2d(SwerveDrivetrainConstants.MODULE_LOCATION_X, -SwerveDrivetrainConstants.MODULE_LOCATION_Y));
	// private final SwerveModule swerveModuleBL = new SwerveModule(
    //         SwerveModuleConstants.VELOCITY_MOTOR_ID_BL,
	// 		SwerveModuleConstants.ANGULAR_MOTOR_ID_BL,
	// 		SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BL,
	// 		SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BL,
    //         new Translation2d(-SwerveDrivetrainConstants.MODULE_LOCATION_X, SwerveDrivetrainConstants.MODULE_LOCATION_Y));
	// private final SwerveModule swerveModuleBR = new SwerveModule(
    //         SwerveModuleConstants.VELOCITY_MOTOR_ID_BR,
	// 		SwerveModuleConstants.ANGULAR_MOTOR_ID_BR,
	// 		SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BR,
	// 		SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BR,
    //         new Translation2d(-SwerveDrivetrainConstants.MODULE_LOCATION_X, -SwerveDrivetrainConstants.MODULE_LOCATION_Y));

    private final AHRS gyro = new AHRS(I2C.Port.kMXP);

    // private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(gyro, swerveModuleFL, swerveModuleFR, swerveModuleBL, swerveModuleBR);

    private final Vision vision = new Vision("Arducam_OV9281_USB_Camera");

    // Create joysticks
    private final CommandJoystick driverJoystick = new CommandJoystick(DriverConstants.DRIVER_JOYSTICK_PORT);
    // private final CommandJoystick operatorJoystick = new CommandJoystick(OperatorConstants.OPERATOR_JOYSTICK_PORT);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // SwerveDriveJoystickControl control = new SwerveDriveJoystickControl(drivetrain, driverJoystick);
        // drivetrain.setDefaultCommand(control);

        // Configure the trigger bindings
        configureBindings();
    }

    /** Use this method to define your trigger->command mappings. */
    private void configureBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;
    }
}
