package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.subsystems.Arm;

public final class Autos {
	/** Example static factory for an autonomous command. */
	public static Command testingAuto(SwerveDrivetrain drivetrain) {
		return Commands.sequence(
				new AutoDriveTo(drivetrain, new Translation2d(1, 0))
		// new WaitCommand(1),
		// new AutoDriveTo(drivetrain, new Translation2d(-1, 0))
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
