package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.IntakeShooter;

/**
 * This class just contains a bunch of auto-modes. Do not call this class
 * itself.
 */
public final class Autos {

	final static double driveDistanceForNote1 = 1.68;

	/** Example static factory for an autonomous command. */
	public static Command driveAuto(SwerveDrivetrain drivetrain, Translation2d translation) {
		return Commands.sequence(
				new AutoDriveTo(drivetrain, translation));
	}

	/** Linden did this */
	public static Command startingAuto(SwerveDrivetrain drivetrain, Arm arm) {

		return Commands.sequence(
				new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES),
				new AutoDriveTo(drivetrain, new Translation2d(driveDistanceForNote1, 0)));
	}

	public static Command shootStartingAuto(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter,
			RobotContainer cRobotContainer) {
		return Commands.sequence(
				shootInSpeaker(drivetrain, arm, shooter, cRobotContainer),

				new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES),
				new AutoDriveTo(drivetrain, new Translation2d(driveDistanceForNote1, 0)),

				new SpinIntakeGrabbers(shooter, 0),
				new SpinFlywheelShooter(shooter, 0));
	}

	/**
	 * This factory creates a new command sequence that picks shoots a preloaded
	 * note and picks up another one before shooting it as well.
	 *
	 * It also pulls both hangers down.
	 *
	 * @author Linden, Aceius E.
	 * @param drivetrain The robot drivetrain
	 * @param arm        The main arm
	 * @param shooter    The shooter
	 * @return Command schedule instructions
	 */
	public static Command shoot2StartingAuto(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter,
			RobotContainer cRobotContainer) {

		return Commands.sequence(
				shootInSpeaker(drivetrain, arm, shooter, cRobotContainer),
				new SpinFlywheelShooter(shooter, -0.1),
				Commands.parallel(
						intakeFromFloorStart(arm, shooter),
						new AutoDriveTo(drivetrain, new Translation2d(driveDistanceForNote1, 0))),
				new WaitCommand(0.25),
				intakeFromFloorEnd(arm, shooter),
				Commands.race(
						Commands.waitSeconds(3),
						new AutoDriveTo(drivetrain, new Translation2d(-driveDistanceForNote1, 0))),
				shootInSpeaker(drivetrain, arm, shooter, cRobotContainer));
	}

	public static Command dropInAmp(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter) {
		return dropInAmp(drivetrain, arm, shooter, null, null);
	}

	public static Command dropInAmp(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Vision vision,
			SendableChooser<Alliance> allyChooser) {

		final int RED_TAG_ID = 5;
		final int BLUE_TAG_ID = 6;

		final double distanceToShootFrom = 0.1;
		final double distanceToAlignAt = 1.5;

		final double minSpinUpTimeSeconds = 0.5;

		// Say whether to use vision
		// final BooleanSupplier shouldUseVisionSupplier = () -> (vision != null &&
		// vision.isEnabled());
		final BooleanSupplier shouldUseVisionSupplier = () -> false;

		// Go to tag 6 if on blue or 5 if on red
		final IntSupplier tagSupplier = () -> ((allyChooser == null || allyChooser.getSelected() == Alliance.Blue)
				? BLUE_TAG_ID
				: RED_TAG_ID);

		// Rotate to 90 if on blue or -90 if on red
		final Supplier<Rotation2d> rotationSupplier = () -> (Rotation2d.fromDegrees(
				(allyChooser == null || allyChooser.getSelected() == Alliance.Blue) ? -90 : 90));

		// Always run if we are not using vision
		// If we are using vision, then check if we can see the target tag
		final BooleanSupplier shouldRunSupplier = () -> (!shouldUseVisionSupplier.getAsBoolean()
				|| vision.getTransformToTag(tagSupplier.getAsInt()) != null);

		return Commands.sequence(
				new AlignAtTagWithX(drivetrain, vision, tagSupplier, distanceToShootFrom, rotationSupplier)
						.onlyIf(shouldUseVisionSupplier),
				Commands.parallel(
						new AutoDriveTo(drivetrain, new Translation2d(-distanceToAlignAt + distanceToShootFrom, 0))
								.onlyIf(shouldUseVisionSupplier),
						new ArmRotateTo(arm, ArmConstants.ARM_AMP_SHOOTING_DEGREES),
						new SpinFlywheelShooter(shooter, IntakeShooterConstants.FLYWHEEL_SHOOTER_SPEED_AMP),
						new WaitCommand(minSpinUpTimeSeconds)),
				new SpinIntakeGrabbers(shooter, IntakeShooterConstants.INTAKE_GRABBER_SPEED_AMP),
				new WaitCommand(0.2),
				new SpinFlywheelShooter(shooter, 0),
				new SpinIntakeGrabbers(shooter, 0),
				new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES))
				.onlyIf(shouldRunSupplier);
	}

	public static Command shootInSpeaker(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter,
			RobotContainer cRobotContainer) {
		return shootInSpeaker(drivetrain, arm, shooter, null, null, cRobotContainer);
	}

	// Code quality really going down hill here, but whatever
	public static Command shootInSpeaker(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Vision vision,
			SendableChooser<Alliance> allyChooser, RobotContainer robotContainer) {

		final int RED_TAG_ID = 4;
		final int BLUE_TAG_ID = 7;

		final double distanceFromTag = 1.55;
		final double spacingFromPoint = 0.25;

		final double minSpinUpTimeSeconds = 1;

		// Say whether to use vision
		final BooleanSupplier shouldUseVisionSupplier = () -> (vision != null && vision.isEnabled());

		// Go to tag 7 if on blue or 4 if on red
		final IntSupplier tagSupplier = () -> ((allyChooser == null || allyChooser.getSelected() == Alliance.Blue)
				? BLUE_TAG_ID
				: RED_TAG_ID);

		// Always run if we are not using vision
		// If we are using vision, then check if we can see the target tag
		final BooleanSupplier shouldRunSupplier = () -> (!shouldUseVisionSupplier.getAsBoolean()
				|| vision.getTransformToTag(tagSupplier.getAsInt()) != null);

		return Commands.parallel(Commands.sequence(
				new SpinFlywheelShooter(shooter, IntakeShooterConstants.FLYWHEEL_SHOOTER_SPEED_SPEAKER),
				Commands.parallel(
						Commands.sequence(
								new AlignAtTagWithX(drivetrain, vision, tagSupplier, distanceFromTag + spacingFromPoint,
										() -> new Rotation2d()),
								new AutoDriveTo(drivetrain, new Translation2d(-spacingFromPoint, 0)))
								.onlyIf(shouldUseVisionSupplier),
						new WaitCommand(minSpinUpTimeSeconds),
						new ArmRotateTo(arm,
								ArmConstants.ARM_SPEAKER_SHOOTING_DEGREES
										+ SmartDashboard.getNumber("shootOffset", 0))),
				new SpinIntakeGrabbers(shooter, IntakeShooterConstants.INTAKE_GRABBER_SPEED_SPEAKER),
				new WaitCommand(0.3),
				new SpinFlywheelShooter(shooter, 0),
				new SpinIntakeGrabbers(shooter, 0),
				new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES)),
				DriverConstants.ENABLE_RUMBLE ? new SetControllerRumbleFor(robotContainer.driverXboxRaw, 3, 1)
						: Commands.sequence(),
				DriverConstants.ENABLE_RUMBLE ? new SetControllerRumbleFor(robotContainer.operatorXboxRaw, 3, 1)
						: Commands.sequence()
		// if rumble isn't enabled pass an empty sequence instead
		).onlyIf(shouldRunSupplier);
	}

	public static Command intakeFromFloorStart(Arm arm, IntakeShooter shooter) {
		return Commands.sequence(
				new SpinFlywheelShooter(shooter, 0.1),
				new SpinIntakeGrabbers(shooter, IntakeShooterConstants.INTAKE_GRABBER_SPEED_SPEAKER),
				new ArmRotateTo(arm, Constants.ArmConstants.ARM_INTAKE_DEGREES));
	}

	public static Command intakeFromFloorEnd(Arm arm, IntakeShooter shooter) {
		return Commands.sequence(
				new SpinFlywheelShooter(shooter, 0),
				new SpinIntakeGrabbers(shooter, -1),
				new WaitCommand(0.01),
				new SpinIntakeGrabbers(shooter, 0),
				new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES));
	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
