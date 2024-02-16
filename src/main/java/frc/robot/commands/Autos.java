package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.subsystems.Arm;

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

	public static Command autoStart(SwerveDrivetrain drivetrain) {

		return Commands.sequence(
				new AutoDriveTo(drivetrain, new Translation2d(0.75946, 0)),
				new WaitCommand(1), // pick up note
				new AutoDriveTo(drivetrain, new Translation2d(-0.757682, 0)),
				new WaitCommand(1), // shot note
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(38.048)),
				new AutoDriveTo(drivetrain, new Translation2d(66.362, 0)),
				new WaitCommand(1), // pick up note
				new AutoDriveTo(drivetrain, new Translation2d(-66.362, 0)),
				new WaitCommand(1), // shot note
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-76.096)),
				new AutoDriveTo(drivetrain, new Translation2d(66.362, 0)),
				new WaitCommand(1), // pick up note
				new AutoDriveTo(drivetrain, new Translation2d(-66.362, 0)),
				new WaitCommand(1) // shot note
		// new AutoRotateTo(drivetrain, Rotation2d.fromDegrees())
		);
	}

	public static Command autoStartBackUpRed(SwerveDrivetrain drivetrain) {

		return Commands.sequence(
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-79.139)), // normal start
				new AutoDriveTo(drivetrain, new Translation2d(4.035, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-7.505)), // starts 4.035 meters to the left of the
				new AutoDriveTo(drivetrain, new Translation2d(6.291, 0)),
				new WaitCommand(1), // pick up note 50%
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(90)),
				new AutoDriveTo(drivetrain, new Translation2d(1.569, 0)),
				new WaitCommand(1), // pick up note 50%
				new AutoDriveTo(drivetrain, new Translation2d(-1.569, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-90)),
				new AutoDriveTo(drivetrain, new Translation2d(-6.291, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-7.505)),
				new AutoDriveTo(drivetrain, new Translation2d(4.035, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-79.139)));
	}

	public static Command autoStartBackUpBlue(SwerveDrivetrain drivetrain) {

		return Commands.sequence(
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(79.139)), // normal start
				new AutoDriveTo(drivetrain, new Translation2d(-4.035, 0)),

				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(7.505)), // starts 4.035 meters to the right of the
				new AutoDriveTo(drivetrain, new Translation2d(-6.291, 0)),
				new WaitCommand(1), // pick up note 50%
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-90)),
				new AutoDriveTo(drivetrain, new Translation2d(-1.569, 0)),
				new WaitCommand(1), // pick up note 50%
				new AutoDriveTo(drivetrain, new Translation2d(1.569, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(90)),
				new AutoDriveTo(drivetrain, new Translation2d(6.291, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(7.505)),
				new AutoDriveTo(drivetrain, new Translation2d(-4.035, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(79.139)));
	}

	public static Command autoStartBackUpBackUpRed(SwerveDrivetrain drivetrain) {

		return Commands.sequence(
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-79.139)),
				new AutoDriveTo(drivetrain, new Translation2d(4.035, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-7.505)),
				new AutoDriveTo(drivetrain, new Translation2d(6.291, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(92.250)),
				new AutoDriveTo(drivetrain, new Translation2d(3.101, 0)),
				new WaitCommand(1), // pick up note 50%
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-2.250)),
				new AutoDriveTo(drivetrain, new Translation2d(1.676, 0)),
				new WaitCommand(1), // pick up note 50%
				new AutoDriveTo(drivetrain, new Translation2d(-1.676, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(2.250)),
				new AutoDriveTo(drivetrain, new Translation2d(-3.101, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-92.250)),
				new AutoDriveTo(drivetrain, new Translation2d(-6.291, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(7.505)),
				new AutoDriveTo(drivetrain, new Translation2d(-4.035, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(79.139)),
				new WaitCommand(1) // shot note
		);
	}

	public static Command autoStartBackUpBackUpBlue(SwerveDrivetrain drivetrain) {

		return Commands.sequence(
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(79.139)),
				new AutoDriveTo(drivetrain, new Translation2d(-4.035, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(7.505)),
				new AutoDriveTo(drivetrain, new Translation2d(-6.291, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-92.250)),
				new AutoDriveTo(drivetrain, new Translation2d(-3.101, 0)),
				new WaitCommand(1), // pick up note 50%
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(2.250)),
				new AutoDriveTo(drivetrain, new Translation2d(-1.676, 0)),
				new WaitCommand(1), // pick up note 50%
				new AutoDriveTo(drivetrain, new Translation2d(1.676, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-2.250)),
				new AutoDriveTo(drivetrain, new Translation2d(3.101, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(92.250)),
				new AutoDriveTo(drivetrain, new Translation2d(6.291, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-7.505)),
				new AutoDriveTo(drivetrain, new Translation2d(4.035, 0)),
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(-79.139)),
				new WaitCommand(1) // shot note
		);
	}

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
