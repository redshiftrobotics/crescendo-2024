package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.subsystems.Arm;

/**
 * This class just contains a bunch of auto-modes. Do not call this class
 * itself.
 */
public final class Autos {
	/** Example static factory for an autonomous command. */
	public static Command testingAuto(SwerveDrivetrain drivetrain) {
		return Commands.sequence(
				new AutoDriveTo(drivetrain, new Translation2d(1.01346, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(5))
		// new WaitCommand(1),
		// new AutoDriveTo(drivetrain, new Translation2d(-1, 0))
		);
	}

	public static Command autoStart(SwerveDrivetrain drivetrain) { // Main AUTO. Goes for the three notes infront of the
																	// speaker

		return Commands.sequence(
				new AutoDriveTo(drivetrain, new Translation2d(0.75946, 0)), // middle note
				new WaitCommand(1), // fire intake 100%
				new AutoDriveTo(drivetrain, new Translation2d(-0.757682, 0)),
				new WaitCommand(1), // shoot note into speaker
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(38.048)), // bottom note
				new AutoDriveTo(drivetrain, new Translation2d(66.362, 0)),
				new WaitCommand(1), // fire intake 100%
				new AutoDriveTo(drivetrain, new Translation2d(-66.362, 0)),
				new WaitCommand(1), // shoot note into speaker
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-76.096)),
				new AutoDriveTo(drivetrain, new Translation2d(66.362, 0)),
				new WaitCommand(1), // fire intake 100%
				new AutoDriveTo(drivetrain, new Translation2d(-66.362, 0)),
				new WaitCommand(1) // shoot note into speaker
		// new AutoRotateTo(drivetrain, Rotation2d.fromDegrees())
		);
	}

	public static Command autoStartBackUpRed(SwerveDrivetrain drivetrain) { // goes for the top 2 notes in the middle of
																			// the field starting from red side

		return Commands.sequence(
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-79.139)), // normal start is directly infront of
																				// speaker
				new AutoDriveTo(drivetrain, new Translation2d(4.035, 0)),

				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-7.505)), // starts 4.035 meters to the left of the
																				// speaker (facing towards center of
																				// field)
				new AutoDriveTo(drivetrain, new Translation2d(6.291, 0)),
				new WaitCommand(1), // fire intake 50%
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(90)),
				new AutoDriveTo(drivetrain, new Translation2d(1.569, 0)),
				new WaitCommand(1), // fire intake 100%
				new AutoDriveTo(drivetrain, new Translation2d(-1.569, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-90)),
				new AutoDriveTo(drivetrain, new Translation2d(-6.291, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-7.505)),
				new AutoDriveTo(drivetrain, new Translation2d(4.035, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-79.139)),
				new WaitCommand(1) // shoot note into speaker

		);
	}

	public static Command autoStartBackUpBlue(SwerveDrivetrain drivetrain) { // goes for the top 2 notes in the middle
																				// of the field starting from blue side

		return Commands.sequence(
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(79.139)), // normal start is directly infront of
																				// speaker
				new AutoDriveTo(drivetrain, new Translation2d(-4.035, 0)),

				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(7.505)), // starts 4.035 meters to the right of the
																				// speaker (facing towards center of
																				// field)
				new AutoDriveTo(drivetrain, new Translation2d(-6.291, 0)),
				new WaitCommand(1), // fire intake 50%
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-90)),
				new AutoDriveTo(drivetrain, new Translation2d(-1.569, 0)),
				new WaitCommand(1), // fire intake 100%
				new AutoDriveTo(drivetrain, new Translation2d(1.569, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(90)),
				new AutoDriveTo(drivetrain, new Translation2d(6.291, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(7.505)),
				new AutoDriveTo(drivetrain, new Translation2d(-4.035, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(79.139)),
				new WaitCommand(1) // shoot note into speaker

		);
	}

	public static Command autoStartBackUpBackUpRed(SwerveDrivetrain drivetrain) { // goes for the center and one lower
																					// note in the center of the field
																					// from the red side

		return Commands.sequence(
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-79.139)), // normal start
				new AutoDriveTo(drivetrain, new Translation2d(4.035, 0)),

				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-7.505)), // starts 4.035 meters to the left of the
																				// speaker (facing towards center of
																				// field)
				new AutoDriveTo(drivetrain, new Translation2d(6.291, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(92.250)),
				new AutoDriveTo(drivetrain, new Translation2d(3.101, 0)),
				new WaitCommand(1), // fire intake 50%
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-2.250)),
				new AutoDriveTo(drivetrain, new Translation2d(1.676, 0)),
				new WaitCommand(1), // fire intake 100%
				new AutoDriveTo(drivetrain, new Translation2d(-1.676, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(2.250)),
				new AutoDriveTo(drivetrain, new Translation2d(-3.101, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-92.250)),
				new AutoDriveTo(drivetrain, new Translation2d(-6.291, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(7.505)),
				new AutoDriveTo(drivetrain, new Translation2d(-4.035, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(79.139)),
				new WaitCommand(1) // shoot note
		);
	}

	public static Command autoStartBackUpBackUpBlue(SwerveDrivetrain drivetrain) { // goes for the center and one lower
																					// note in the center of the field
																					// from the blue side

		return Commands.sequence(
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(79.139)), // normal startS
				new AutoDriveTo(drivetrain, new Translation2d(-4.035, 0)),

				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(7.505)), // starts 4.035 meters to the right of the
																				// speaker (facing towards center of
																				// field)
				new AutoDriveTo(drivetrain, new Translation2d(-6.291, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-92.250)),
				new AutoDriveTo(drivetrain, new Translation2d(-3.101, 0)),
				new WaitCommand(1), // fire intake 50%
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(2.250)),
				new AutoDriveTo(drivetrain, new Translation2d(-1.676, 0)),
				new WaitCommand(1), // fire intake 100%
				new AutoDriveTo(drivetrain, new Translation2d(1.676, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-2.250)),
				new AutoDriveTo(drivetrain, new Translation2d(3.101, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(92.250)),
				new AutoDriveTo(drivetrain, new Translation2d(6.291, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-7.505)),
				new AutoDriveTo(drivetrain, new Translation2d(4.035, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-79.139)),
				new WaitCommand(1) // shoot note
		);
	}

	public static Command rotateTo90Auto(SwerveDrivetrain drivetrain) {
		return Commands.sequence(
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(90), true));
	}

	public static Command rotateBy90Auto(SwerveDrivetrain drivetrain) {
		return Commands.sequence(
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(90), false));
	}

	public static Command rotateToNegative90Auto(SwerveDrivetrain drivetrain) {
		return Commands.sequence(
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-90), true));
	}

	public static Command rotateByNegative90Auto(SwerveDrivetrain drivetrain) {
		return Commands.sequence(
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-90), false));
	}

	public static Command rotateBy10Auto(SwerveDrivetrain drivetrain) {
		return Commands.sequence(new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(10), true));
	}

	/** Auto-mode that attempts to follow an april tag. */
	public static Command tagFollowAuto(SwerveDrivetrain drivetrain, Vision camera, Integer tagId) {
		return new FollowTag(drivetrain, camera, new Translation2d(1, 0), tagId, null);
	}

	/** Linden did this */
	public static Command startingAuto(Arm arm, SwerveDrivetrain drivetrain, boolean invertY) {

		// assumes start position in corner
		double invert = 1;
		if (invertY) {
			invert = -1;
		}

		return Commands.sequence(
				// 2.9, 0.2 and 1.2 are not arbitrary, they move the robot so that the note is
				// right in front; 0.05 can be changed, it's for the amount of extra spacing
				// that we want
				new AutoDriveTo(drivetrain,
						new Translation2d(2.9 - 0.2 - SwerveDrivetrainConstants.MODULE_LOCATION_X - 0.05,
								invert * (1.2 - SwerveDrivetrainConstants.MODULE_LOCATION_Y))),

				new AutoRotateTo(drivetrain, new Rotation2d(Math.PI / -2 * invert)));
	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
