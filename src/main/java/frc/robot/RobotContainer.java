package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.commands.Autos;
import frc.robot.commands.ArmRotateTo;
import frc.robot.commands.ChassisRemoteControl;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Vision;
import frc.robot.inputs.ChassisDriveInputs;
import frc.robot.inputs.OptionButtonInput;
import frc.robot.inputs.OptionButtonInput.ActivationMode;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import java.util.List;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic
 * should actually be handled in the {@link Robot} periodic methods (other than
 * the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

	// The robot's subsystems and commands are defined here...
	private final SwerveModule swerveModuleFL = new SwerveModule(
			SwerveModuleConstants.VELOCITY_MOTOR_ID_FL,
			SwerveModuleConstants.ANGULAR_MOTOR_ID_FL,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FL,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FL,
			new Translation2d(SwerveDrivetrainConstants.MODULE_LOCATION_X,
					SwerveDrivetrainConstants.MODULE_LOCATION_Y));
	private final SwerveModule swerveModuleFR = new SwerveModule(
			SwerveModuleConstants.VELOCITY_MOTOR_ID_FR,
			SwerveModuleConstants.ANGULAR_MOTOR_ID_FR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FR,
			new Translation2d(SwerveDrivetrainConstants.MODULE_LOCATION_X,
					-SwerveDrivetrainConstants.MODULE_LOCATION_Y));
	private final SwerveModule swerveModuleBL = new SwerveModule(
			SwerveModuleConstants.VELOCITY_MOTOR_ID_BL,
			SwerveModuleConstants.ANGULAR_MOTOR_ID_BL,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BL,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BL,
			new Translation2d(-SwerveDrivetrainConstants.MODULE_LOCATION_X,
					SwerveDrivetrainConstants.MODULE_LOCATION_Y));
	private final SwerveModule swerveModuleBR = new SwerveModule(
			SwerveModuleConstants.VELOCITY_MOTOR_ID_BR,
			SwerveModuleConstants.ANGULAR_MOTOR_ID_BR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BR,
			new Translation2d(-SwerveDrivetrainConstants.MODULE_LOCATION_X,
					-SwerveDrivetrainConstants.MODULE_LOCATION_Y));

	private final AHRS gyro = new AHRS();
    private final Arm arm = new Arm(
        ArmConstants.LEFT_MOTOR_ID,
        ArmConstants.RIGHT_MOTOR_ID,
        ArmConstants.RIGHT_ENCODER_ID,
		ArmConstants.ARE_MOTORS_REVERSED);


	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(gyro, swerveModuleFL, swerveModuleFR,
			swerveModuleBL, swerveModuleBR);

	// Create joysticks
	private final CommandJoystick driverJoystick = new CommandJoystick(DriverConstants.DRIVER_JOYSTICK_PORT);
	// private final CommandJoystick operatorJoystick = new
	// CommandJoystick(OperatorConstants.OPERATOR_JOYSTICK_PORT);

	// private final CommandXboxController xboxController = new
	// CommandXboxController(0);
	private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

	private final Vision vision = new Vision(VisionConstants.CAMERA_NAME, VisionConstants.CAMERA_POSE);


	private final ArmRotateTo armToIntake = new ArmRotateTo(arm, ArmConstants.ARM_INTAKE_DEGREES);
	private final ArmRotateTo armToAmp = new ArmRotateTo(arm, ArmConstants.ARM_AMP_SHOOTING_DEGREES);
	private final ArmRotateTo armToSpeaker = new ArmRotateTo(arm, ArmConstants.ARM_SPEAKER_SHOOTING_DEGREES);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		autoChooser.setDefaultOption("Testing Auto", Autos.testingAuto(drivetrain));
		autoChooser.addOption("Follow Tag", Autos.tagFollowAuto(drivetrain, vision, 1));
		autoChooser.addOption("Rotate by 90", Autos.rotateBy90Auto(drivetrain));
		autoChooser.addOption("Rotate to 90", Autos.rotateTo90Auto(drivetrain));
		autoChooser.addOption("Rotate by -90", Autos.rotateByNegative90Auto(drivetrain));
		autoChooser.addOption("Rotate to -90", Autos.rotateToNegative90Auto(drivetrain));
		autoChooser.addOption("Rotate by 10", Autos.rotateBy10Auto(drivetrain));
		SmartDashboard.putData("Auto Chooser", autoChooser);

		configureBindings();

		setUpDriveController();

		PortForwarder.add(5800, "photonvision.local", 5800);
	}

	public void setUpDriveController() {
		// Create joysticks
		final GenericHID genericHID = new GenericHID(DriverConstants.DRIVER_JOYSTICK_PORT);
		final HIDType genericHIDType = genericHID.getType();

		final CommandJoystick operatorJoystick = new CommandJoystick(DriverConstants.OPERATOR_JOYSTICK_PORT);

		SmartDashboard.putString("Drive Controller", genericHIDType.toString());
		SmartDashboard.putString("Bot Name", Constants.currentBot.toString() + " - " + Constants.serialNumber);

		drivetrain.removeDefaultCommand();

		ChassisDriveInputs inputs;
		OptionButtonInput preciseModeButton, boostModeButton, fieldRelativeButton;

		if (genericHIDType.equals(GenericHID.HIDType.kHIDJoystick)) {
			final CommandJoystick joystick = new CommandJoystick(genericHID.getPort());

			inputs = new ChassisDriveInputs(
					joystick::getX, -1,
					joystick::getY, -1,
					joystick::getTwist, -1,
					Constants.DriverConstants.DEAD_ZONE);

			preciseModeButton = new OptionButtonInput(joystick, 2, ActivationMode.TOGGLE);
			boostModeButton = new OptionButtonInput(joystick, 1, ActivationMode.HOLD);
			fieldRelativeButton = new OptionButtonInput(joystick, 3, ActivationMode.TOGGLE);

			//This bypasses arm remote control, arm remote control is incompatible with autonomous commands
			operatorJoystick.button(4).onTrue(armToIntake);
			operatorJoystick.button(5).onTrue(armToAmp);
			operatorJoystick.button(6).onTrue(armToSpeaker);

			joystick.button(9).onTrue(Commands.run(drivetrain::brakeMode, drivetrain));
			joystick.button(10).onTrue(Commands.run(drivetrain::toDefaultStates, drivetrain));
		} else {
			final CommandXboxController xbox = new CommandXboxController(genericHID.getPort());

			inputs = new ChassisDriveInputs(
					xbox::getLeftY, +1,
					xbox::getLeftX, +1,
					xbox::getRightX, -1,
					Constants.DriverConstants.DEAD_ZONE);

			preciseModeButton = new OptionButtonInput(xbox::b, ActivationMode.TOGGLE);
			boostModeButton = new OptionButtonInput(xbox::leftStick, ActivationMode.HOLD);
			fieldRelativeButton = new OptionButtonInput(xbox::povUp, ActivationMode.TOGGLE);
		}

		drivetrain.setDefaultCommand(
				new ChassisRemoteControl(drivetrain, inputs, preciseModeButton, boostModeButton, fieldRelativeButton));
	}

	/** Use this method to define your trigger->command mappings. */
	private void configureBindings() {
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return autoChooser.getSelected();
	}
}
