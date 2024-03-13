package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.IntakeShooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.HangConstants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.inputs.ChassisDriveInputs;
import frc.robot.subsystems.hang.Hang;

/**
 * This class just contains a bunch of auto-modes. Do not call this class
 * itself.
 */
public final class Autos {
	/**
	 * The translation to the preliminary position for third note pickup
	 * <p>
	 * Relative to the robot which will be facing away from the speaker
	 */
	private static final Translation2d rightNotePickup = new Translation2d(
			-(Constants.BOT_WIDTH / 2) - Units.inchesToMeters(77.875 / 2), Units.inchesToMeters(39));
	private static final double speakerDepth = Units.inchesToMeters(36.125);

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
						new AutoDriveTo(drivetrain, new Translation2d(7, 0))),
				new PullHangerDown(rightHang, HangConstants.SPEED),
				new PullHangerDown(leftHang, HangConstants.SPEED));
	}

	public static Command shootStartingAuto(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Hang leftHang,
			Hang rightHang) {
		return Commands.sequence(
				shootInSpeaker(drivetrain, arm, shooter, null, null),

				startingAuto(drivetrain, arm, leftHang, rightHang),
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
		final double driveDistanceForNote1 = -1.59;

		return Commands.parallel(
				Commands.sequence(
						shootInSpeaker(drivetrain, arm, shooter, null, null),
						new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES),
						Commands.parallel(
								new AutoDriveTo(drivetrain, new Translation2d(driveDistanceForNote1, 0)),
								new SpinIntakeGrabbers(shooter, IntakeShooterConstants.INTAKE_GRABBER_SPEED_SPEAKER)),
						new SpinIntakeGrabbers(shooter, 0),
						new AutoDriveTo(drivetrain, new Translation2d(-driveDistanceForNote1, 0)),
						shootInSpeaker(drivetrain, arm, shooter, null, null)),
				new PullHangerDown(rightHang, HangConstants.SPEED),
				new PullHangerDown(leftHang, HangConstants.SPEED));
	}

	public static Command shoot3StartingAuto(SwerveDrivetrain drivetrain, Arm arm, IntakeShooter shooter, Vision vision,
			Hang leftHanger, Hang rightHanger, ChassisDriveInputs inputs) throws Exception {
		final Optional<Alliance> ally = DriverStation.getAlliance();

		if (!vision.isEnabled())
			throw new UnsupportedOperationException("This auto requires vision!");
		if (ally.isEmpty())
			throw new Exception("DriverStation.getAlliance is not present. Set the alliance in the driver station.");

		boolean redAlliance = ally.get() == Alliance.Red;
		final int speakerTagId = redAlliance ? 4 : 7;
		final Translation2d note3Translation = new Translation2d(
				redAlliance ? rightNotePickup.getX() : -rightNotePickup.getY(),
				rightNotePickup.getY());

		return Commands.sequence(
				shoot2StartingAuto(drivetrain, arm, shooter, leftHanger, rightHanger),
				intakeFromFloorStart(arm, shooter),
				new AutoDriveTo(drivetrain, note3Translation),
				new AutoDriveTo(drivetrain, new Translation2d(0, note3Translation.getY())),
				intakeFromFloorEnd(arm, shooter),
				new AutoDriveTo(drivetrain, note3Translation.times(-1)),
				new FollowTag(drivetrain, vision, speakerTagId,
						new Translation2d(-speakerDepth - Constants.BOT_WIDTH / 2, 0)),
				shootInSpeaker(drivetrain, arm, shooter, null, inputs));

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
				new ArmRotateTo(arm, ArmConstants.ARM_AMP_SHOOTING_DEGREES).alongWith(
						new SpinFlywheelShooter(shooter, IntakeShooterConstants.FLYWHEEL_SHOOTER_SPEED_AMP),
						new WaitCommand(1)),
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
						Commands.sequence(
								new SpinFlywheelShooter(shooter, IntakeShooterConstants.FLYWHEEL_SHOOTER_SPEED_SPEAKER),
								new WaitCommand(0.5)),
						new ArmRotateTo(arm, ArmConstants.ARM_SPEAKER_SHOOTING_DEGREES, 1)),
				new SpinIntakeGrabbers(shooter, IntakeShooterConstants.INTAKE_GRABBER_SPEED_SPEAKER),
				new WaitCommand(0.2),
				new SpinFlywheelShooter(shooter, 0),
				new SpinIntakeGrabbers(shooter, 0));
	}

	public static Command intakeFromFloorStart(Arm arm, IntakeShooter shooter) {
		return Commands.sequence(
				new SpinIntakeGrabbers(shooter, IntakeShooterConstants.INTAKE_GRABBER_SPEED_SPEAKER),
				new ArmRotateTo(arm, Constants.ArmConstants.ARM_INTAKE_DEGREES));
	}

	public static Command intakeFromFloorEnd(Arm arm, IntakeShooter shooter) {
		return Commands.sequence(
				new SpinIntakeGrabbers(shooter, -1),
				new WaitCommand(0.04),
				new SpinIntakeGrabbers(shooter, 0));
	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
