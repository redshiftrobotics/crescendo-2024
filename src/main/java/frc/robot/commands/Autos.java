package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.IntakeShooter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.SwerveDrivetrainConstants;

/**
 * This class just contains a bunch of auto-modes. Do not call this class
 * itself.
 */
public final class Autos {
	/** Example static factory for an autonomous command. */
	public static Command driveAuto(SwerveDrivetrain drivetrain, double meters) {
		return Commands.sequence(
				new AutoDriveTo(drivetrain, new Translation2d(meters, meters)));
	}

	public static Command rotateTestAuto(SwerveDrivetrain drivetrain, double degrees, boolean fieldRelative) {
		return Commands.sequence(
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(90), fieldRelative));
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

	public static Command dropInAmp(Arm arm, IntakeShooter shooter) {
		return Commands.sequence(
				new ArmRotateTo(arm, Constants.ArmConstants.ARM_AMP_SHOOTING_DEGREES),
				new SpinIntakeFlywheels(shooter, Constants.IntakeShooterConstants.FLYWHEEL_SPEED_AMP),
				new WaitCommand(1),
				new SpinIntakeWheels(shooter, Constants.IntakeShooterConstants.WHEEL_SPEED_AMP),
				new WaitCommand(1),
				new SpinIntakeFlywheels(shooter, 0),
				new SpinIntakeWheels(shooter, 0));
	}

	public static Command dropInSpeaker(Arm arm, IntakeShooter shooter) {
		return Commands.sequence(
				new ArmRotateTo(arm, Constants.ArmConstants.ARM_SPEAKER_SHOOTING_DEGREES),
				new SpinIntakeFlywheels(shooter, Constants.IntakeShooterConstants.FLYWHEEL_SPEED_SPEAKER),
				new WaitCommand(1),
				new SpinIntakeWheels(shooter, Constants.IntakeShooterConstants.WHEEL_SPEED_SPEAKER),
				new WaitCommand(0.25),
				new SpinIntakeFlywheels(shooter, 0),
				new SpinIntakeWheels(shooter, 0));
	}

	/*
	 * 1 set arm to right position
	 * rotate arm
	 * 
	 */

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
