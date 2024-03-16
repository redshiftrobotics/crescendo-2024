package frc.robot.commands;

import frc.robot.subsystems.ChassisDriveInputs;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.IntakeShooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.HangConstants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.hang.Hang;

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
				shootInSpeaker(drivetrain, arm, shooter, null, null),

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
						shootInSpeaker(drivetrain, arm, shooter, null, null),
						new SpinFlywheelShooter(shooter, -0.1),
						Commands.parallel(
								intakeFromFloorStart(arm, shooter),
								new AutoDriveTo(drivetrain, new Translation2d(driveDistanceForNote1, 0))),
						new WaitCommand(0.25),
						intakeFromFloorEnd(arm, shooter),
						Commands.race(
								Commands.waitSeconds(3),
								new AutoDriveTo(drivetrain, new Translation2d(-driveDistanceForNote1, 0))),
						shootInSpeaker(drivetrain, arm, shooter, null, null)),
				new PullHangerDown(rightHang, HangConstants.SPEED),
				new PullHangerDown(leftHang, HangConstants.SPEED));
	}

	public static Command shootFromSideAuto(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Hang leftHang,
			Hang rightHang, boolean flipped) {

		Optional<Alliance> ally = DriverStation.getAlliance();
		if (ally.isPresent() && ally.get() == Alliance.Red) {
			flipped = !flipped;
		}

		drivetrain.setFrontOffset(Rotation2d.fromDegrees(flipped ? -60 : 60));

		return Commands.parallel(
				Commands.sequence(
						// First note
						shootInSpeaker(drivetrain, arm, shooter, null, null),

						new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES),
						// Line up X to second note
						new AutoDriveTo(drivetrain, new Translation2d(0.25, 0)),
						new AutoRotateTo(drivetrain, new Rotation2d(0), true), 
						
						// Pick up First note
						intakeFromFloorStart(arm, shooter), 
						new AutoDriveTo(drivetrain, new Translation2d(2.15, 0)),
						intakeFromFloorEnd(arm, shooter),

						new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES),
						// Get Back 
						new AutoDriveTo(drivetrain, new Translation2d(-2.15, 0)),
						new AutoRotateTo(drivetrain, new Rotation2d(flipped ? 60 : -60), true),
						new AutoDriveTo(drivetrain, new Translation2d(0.25, 0)),

						// Shoot Second
						shootInSpeaker(drivetrain, arm, shooter, null, null)
				),

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
		// if (!vision.isEnabled())
		// throw new UnsupportedOperationException("This auto requires vision!");

		final int speakerTagId = (alliance == Alliance.Red) ? 4 : 7;
		final Translation2d sideNoteTranslation = new Translation2d(rightNotePickup.getX(),
				(alliance == Alliance.Red) == isAmpSide ? rightNotePickup.getY() : -rightNotePickup.getY());
		return Commands.sequence(
				intakeFromFloorStart(arm, shooter),
				new AutoDriveTo(drivetrain, sideNoteTranslation),
				new AutoDriveTo(drivetrain, new Translation2d(0, sideNoteTranslation.getY())),
				intakeFromFloorEnd(arm, shooter),
				// new AutoDriveTo(drivetrain, sideNoteTranslation.times(-1)),
				new AutoDriveTo(drivetrain,
						new Translation2d(-sideNoteTranslation.getX() * 2, -sideNoteTranslation.getY())),
				// new FollowTag(drivetrain, vision, speakerTagId,
				// new Translation2d(-speakerDepth - Constants.BOT_WIDTH / 2, 0)),
				shootInSpeaker(drivetrain, arm, shooter, null, inputs));
	}

	public static Command shoot3StartingAuto(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Vision vision,
			Hang leftHanger, Hang rightHanger, ChassisDriveInputs inputs) throws Exception {
		final Optional<Alliance> ally = DriverStation.getAlliance();

		// if (!vision.isEnabled())
		// throw new UnsupportedOperationException("This auto requires vision!");
		if (ally.isEmpty())
			throw new Exception("DriverStation.getAlliance is not present. Set the alliance in the driver station.");

		boolean redAlliance = ally.get() == Alliance.Red;

		return Commands.sequence(
				shoot2StartingAuto(drivetrain, arm, shooter, leftHanger, rightHanger),
				shootSideNoteAuto(drivetrain, arm, shooter, vision, ally.get(), true, inputs));

	}

	public static Command shoot4StartingAuto(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Vision vision,
			Hang leftHanger, Hang rightHanger, ChassisDriveInputs inputs) throws Exception {
		final Optional<Alliance> ally = DriverStation.getAlliance();

		// if (!vision.isEnabled())
		// throw new UnsupportedOperationException("This auto requires vision!");
		if (ally.isEmpty())
			throw new Exception("DriverStation.getAlliance is not present. Set the alliance in the driver station.");

		return Commands.sequence(
				shoot2StartingAuto(drivetrain, arm, shooter, leftHanger, rightHanger),
				shootSideNoteAuto(drivetrain, arm, shooter, vision, ally.get(), true, inputs),
				shootSideNoteAuto(drivetrain, arm, shooter, vision, ally.get(), false, inputs));

	}

	public static Command dropInAmp(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Vision vision,
			ChassisDriveInputs inputs) {

		Optional<Alliance> ally = DriverStation.getAlliance();
		if (vision != null && vision.isEnabled() && ally.isPresent()) {

			int tagId = -1;
			Rotation2d rotation = Rotation2d.fromDegrees(90);
			if (ally.get() == Alliance.Red) {
				tagId = 5;
			}
			if (ally.get() == Alliance.Blue) {
				tagId = 6;
			}

			return Commands.sequence(
					new AutoRotateTo(drivetrain, rotation, true),
					new AlignAtTag(drivetrain, vision, tagId, inputs),
					shootInSpeaker(drivetrain, arm, shooter, null, inputs));
		}

		return Commands.sequence(
				Commands.parallel(
					new SpinFlywheelShooterForTime(shooter, IntakeShooterConstants.FLYWHEEL_SHOOTER_SPEED_AMP,
							0.5),
					new ArmRotateTo(arm, ArmConstants.ARM_AMP_SHOOTING_DEGREES)),
				new SpinIntakeGrabbers(shooter, IntakeShooterConstants.INTAKE_GRABBER_SPEED_AMP),
				new WaitCommand(0.2),
				new SpinFlywheelShooter(shooter, 0),
				new SpinIntakeGrabbers(shooter, 0));
	}

	public static Command shootInSpeaker(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Vision vision,
			ChassisDriveInputs inputs) {

		Optional<Alliance> ally = DriverStation.getAlliance();
		if (vision != null && vision.isEnabled() && ally.isPresent()) {

			int tagId = -1;
			Rotation2d rotation = Rotation2d.fromDegrees(0);
			if (ally.get() == Alliance.Red) {
				tagId = 4;
			}
			if (ally.get() == Alliance.Blue) {
				tagId = 7;
			}

			return Commands.sequence(
					new AutoRotateTo(drivetrain, rotation, true),
					new AlignAtTag(drivetrain, vision, tagId, inputs),
					shootInSpeaker(drivetrain, arm, shooter, null, inputs));
		}

		return Commands.sequence(
				Commands.parallel(
						new SpinFlywheelShooterForTime(shooter, IntakeShooterConstants.FLYWHEEL_SHOOTER_SPEED_SPEAKER,
								1.7),
						new ArmRotateTo(arm, ArmConstants.ARM_SPEAKER_SHOOTING_DEGREES)),
				new SpinIntakeGrabbers(shooter, IntakeShooterConstants.INTAKE_GRABBER_SPEED_SPEAKER),
				new WaitCommand(0.2),
				new SpinFlywheelShooter(shooter, 0),
				new SpinIntakeGrabbers(shooter, 0));
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
				new WaitCommand(0.04),
				new SpinIntakeGrabbers(shooter, 0));
	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
