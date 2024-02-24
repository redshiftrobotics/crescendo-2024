package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This class just contains a bunch of auto-modes. Do not call this class
 * itself.
 */
public final class Autos {
	/** Example static factory for an autonomous command. */
	public static Command testingAuto(SwerveDrivetrain drivetrain) {
		return Commands.sequence(
				new AutoRotateTo(drivetrain, Rotation2d.fromDegrees(90), false));
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





	public static Command trajectoryAuto(SwerveDrivetrain drivetrain){


		// Trajectory Config
		final TrajectoryConfig exampleConfig = new TrajectoryConfig(AutoConstants.kMaxAutoVelocitySpeedMetersPerSecond,
					AutoConstants.kMaxAutoRotationSpeedMetersPerSecond).setKinematics(drivetrain.getKinematics());
		// Example Trajectory (1 meter forward then backward)
		final Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
					List.of(new Translation2d(0, 0.5)), new Pose2d(0, 1, new Rotation2d(0)), exampleConfig);
		// Profiled PID Controller for trajectory rotation
		final ProfiledPIDController rotationPidController = new ProfiledPIDController(AutoConstants.kAngularControllerP,
					0, 0, AutoConstants.kRotationControllerConstraints);
		rotationPidController.enableContinuousInput(-Math.PI, Math.PI);
		// SwerveControllerCommand Test (Trajectory Auto Drive)
		final SwerveControllerCommand trajectoryTestCommand = new SwerveControllerCommand(exampleTrajectory,
					drivetrain::getPosition, drivetrain.getKinematics(),
					new PIDController(AutoConstants.kVelocityControllerP, 0, 0),
					new PIDController(AutoConstants.kVelocityControllerP, 0, 0), rotationPidController,
					drivetrain::setSwerveModuleStates, drivetrain);

		return Commands.sequence(new InstantCommand(() -> drivetrain.resetPosition()), trajectoryTestCommand,
					new InstantCommand(() -> drivetrain.setDesiredState(new ChassisSpeeds(0, 0, 0))));

	}
}
