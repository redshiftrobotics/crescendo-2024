package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.HangConstants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.ChassisDriveInputs;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.hang.Hang;
import frc.robot.subsystems.intake.IntakeShooter;

/**
 * This class just contains a bunch of auto-modes. Do not call this class
 * itself.
 */
public final class Autos {

	/**
	 * The translation to the preliminary position a side note pickup
	 * <p>
	 * Relative to the robot which will be facing away from the speaker
	 */
	private static final double speakerDepth = Units.inchesToMeters(36.125);
	private static final Translation2d rightNotePickup = new Translation2d(
			/* (dist from note to speaker front) / 2 - (bot offset from front of speaker) */
			Units.inchesToMeters((114 - speakerDepth) / 2) - (Constants.BOT_WIDTH / 2),
			/* lateral dist to note */
			-Units.inchesToMeters(57));
	final static double driveDistanceForNote1 = 1.1;

	/** Example static factory for an autonomous command. */
	public static Command driveAuto(SwerveDrivetrain drivetrain, Translation2d translation) {
		return Commands.sequence(
				new AutoDriveTo(drivetrain, translation));
	}

	/** Linden did this */
	public static Command startingAuto(SwerveDrivetrain drivetrain, Arm arm, Hang leftHang, Hang rightHang) {

		return Commands.parallel(
				Commands.sequence(
						new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES),
						new AutoDriveTo(drivetrain, new Translation2d(driveDistanceForNote1, 0))),

				new PullHangerDown(rightHang, HangConstants.SPEED),
				new PullHangerDown(leftHang, HangConstants.SPEED));
	}

	public static Command shootStartingAuto(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Hang leftHang,
			Hang rightHang) {
		return Commands.sequence(
				shootInSpeaker(drivetrain, arm, shooter),

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
	 * @param leftHang   The left hanger
	 * @param rightHang  The right hanger
	 * @return Command schedule instructions
	 */
	public static Command shoot2StartingAuto(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Hang leftHang,
			Hang rightHang) {

		return Commands.parallel(
				Commands.sequence(
						shootInSpeaker(drivetrain, arm, shooter),
						new SpinFlywheelShooter(shooter, -0.1),
						Commands.parallel(
								intakeFromFloorStart(arm, shooter),
								new AutoDriveTo(drivetrain, new Translation2d(driveDistanceForNote1, 0))),
						new WaitCommand(0.25),
						intakeFromFloorEnd(arm, shooter),
						Commands.race(
								Commands.waitSeconds(3),
								new AutoDriveTo(drivetrain, new Translation2d(-driveDistanceForNote1, 0))),
						shootInSpeaker(drivetrain, arm, shooter)),
				new PullHangerDown(rightHang, HangConstants.SPEED),
				new PullHangerDown(leftHang, HangConstants.SPEED));
	}

	public static Command shootFromSideAuto(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Hang leftHang,
			Hang rightHang, boolean flipped) {

		return Commands.parallel(
				Commands.sequence(
						// First note
						shootInSpeaker(drivetrain, arm, shooter),

						new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES),
						new AutoDriveTo(drivetrain, new Translation2d(3, 0))),

				new PullHangerDown(rightHang, HangConstants.SPEED),
				new PullHangerDown(leftHang, HangConstants.SPEED));
	}

	public static Command shootFromAmpSideAuto(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter,
			Hang leftHang, Hang rightHang) {
		return shootFromSideAuto(drivetrain, arm, shooter, leftHang, rightHang, false);
	}

	public static Command shootFromFarSideAuto(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter,
			Hang leftHang, Hang rightHang) {
		return shootFromSideAuto(drivetrain, arm, shooter, leftHang, rightHang, true);
	}

	public static Command shootSideNoteAuto(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter,
			Vision vision, Alliance alliance, boolean isAmpSide, ChassisDriveInputs inputs) {
		// If on red, and going for amp side note (right side) keep translation
		// If on red, and going for stage side note (left side) mirror translation y
		// If on blue, and going for amp side note (left side) mirror translation y
		// If on blue, and going for stage side note (right side) keep translation

		final Translation2d sideNoteTranslation = new Translation2d(rightNotePickup.getX(),
				(alliance == Alliance.Red) == isAmpSide ? rightNotePickup.getY() : -rightNotePickup.getY());

		return Commands.sequence(
				intakeFromFloorStart(arm, shooter),
				new AutoDriveTo(drivetrain, sideNoteTranslation),
				new AutoDriveTo(drivetrain, new Translation2d(0, sideNoteTranslation.getY())),
				intakeFromFloorEnd(arm, shooter),
				new AutoDriveTo(drivetrain,
						new Translation2d(-sideNoteTranslation.getX() * 2, -sideNoteTranslation.getY())),
				shootInSpeaker(drivetrain, arm, shooter, null, inputs, alliance));
	}

	public static Command shoot3StartingAuto(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Vision vision,
			Hang leftHanger, Hang rightHanger, ChassisDriveInputs inputs, Alliance ally) {

		return Commands.sequence(
				shoot2StartingAuto(drivetrain, arm, shooter, leftHanger, rightHanger),
				shootSideNoteAuto(drivetrain, arm, shooter, vision, ally, true, inputs));
	}

	public static Command dropInAmp(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter) {
		return dropInAmp(drivetrain, arm, shooter, null, null, null);
	}

	public static Command dropInAmp(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Vision vision,
			ChassisDriveInputs inputs, SendableChooser<Alliance> team) {

		if (vision != null && vision.isEnabled()) {

			Rotation2d rotation = Rotation2d.fromDegrees(90);
			if (team.getSelected() == Alliance.Blue) {
				rotation = Rotation2d.fromDegrees(-90);
			}

			return Commands.sequence(
					new AlignAtTagWithX(drivetrain, vision, new int[] { 5, 6 }, 1, rotation),
					new AutoDriveTo(drivetrain, new Translation2d(-1, 0)),
					dropInAmp(drivetrain, arm, shooter, null, inputs, team));
		}

		return Commands.sequence(
				new ArmRotateTo(arm, ArmConstants.ARM_AMP_SHOOTING_DEGREES).alongWith(
						new SpinFlywheelShooter(shooter, IntakeShooterConstants.FLYWHEEL_SHOOTER_SPEED_AMP),
						new WaitCommand(0.5)),
				new SpinIntakeGrabbers(shooter, IntakeShooterConstants.INTAKE_GRABBER_SPEED_AMP),
				new WaitCommand(0.2),
				new SpinFlywheelShooter(shooter, 0),
				new SpinIntakeGrabbers(shooter, 0),
				new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES));
	}

	public static Command shootInSpeaker(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter) {
		return shootInSpeaker(drivetrain, arm, shooter, null, null, null);
	}

	// Code quality really going down hill here, but ehhh
	public static Command shootInSpeaker(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Vision vision,
			ChassisDriveInputs inputs, Alliance team) {

		if (vision != null && vision.isEnabled()) {

			return Commands.sequence(
					Commands.parallel(
							new SpinFlywheelShooter(shooter, IntakeShooterConstants.FLYWHEEL_SHOOTER_SPEED_SPEAKER),
							new AlignAtTagWithX(drivetrain, vision, new int[] { 7, 4 }, 1.59).raceWith(
									Commands.waitSeconds(5)),
							new WaitCommand(1),
							new ArmRotateTo(arm,
									ArmConstants.ARM_SPEAKER_SHOOTING_DEGREES
											+ SmartDashboard.getNumber("shootOffset", 0))),
					new SpinIntakeGrabbers(shooter, IntakeShooterConstants.INTAKE_GRABBER_SPEED_SPEAKER),
					new WaitCommand(0.3),
					new SpinFlywheelShooter(shooter, 0),
					new SpinIntakeGrabbers(shooter, 0),
					new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES))
					.onlyIf(() -> vision.getTransformToTag(7) != null || vision.getTransformToTag(4) != null);

		}

		return Commands.sequence(
				new SpinFlywheelShooter(shooter, IntakeShooterConstants.FLYWHEEL_SHOOTER_SPEED_SPEAKER),
				Commands.parallel(
						new WaitCommand(1.7),
						new ArmRotateTo(arm,
								ArmConstants.ARM_SPEAKER_SHOOTING_DEGREES
										+ SmartDashboard.getNumber("shootOffset", 0))),
				new SpinIntakeGrabbers(shooter, IntakeShooterConstants.INTAKE_GRABBER_SPEED_SPEAKER),
				new WaitCommand(0.3),
				new SpinFlywheelShooter(shooter, 0),
				new SpinIntakeGrabbers(shooter, 0),
				new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES));
	}

	public static Command shootInSpeakerFar(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter,
			Vision vision) {

		if (vision == null || !vision.isEnabled()) {
			return shootInSpeaker(drivetrain, arm, shooter);
		}

		var tags = new int[] { 7, 4 };

		boolean cancel = true;
		for (int i : tags) {
			if (vision.getTransformToTag(i) != null) {
				cancel = false;
			}
		}

		if (cancel) {
			return Commands.none();
		}

		return Commands.sequence(
				new SpinFlywheelShooter(shooter, IntakeShooterConstants.FLYWHEEL_SHOOTER_SPEED_SPEAKER),
				Commands.parallel(
						new WaitCommand(1.7),
						new AlignAtTagWithX(drivetrain, vision, tags, 7),
						new ArmRotateTo(arm,
								ArmConstants.ARM_SPEAKER_SHOOTING_DEGREES
										+ SmartDashboard.getNumber("shootOffset", 0))),
				new SpinIntakeGrabbers(shooter, IntakeShooterConstants.INTAKE_GRABBER_SPEED_SPEAKER),
				new WaitCommand(0.3),
				new SpinFlywheelShooter(shooter, 0),
				new SpinIntakeGrabbers(shooter, 0),
				new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES));
	}

	public static Command intakeFromFloorStart(Arm arm, IntakeShooter shooter) {
		return Commands.sequence(
				new SpinFlywheelShooter(shooter, -0.1),
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
