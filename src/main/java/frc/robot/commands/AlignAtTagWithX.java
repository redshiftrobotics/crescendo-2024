package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotMovementConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

/** Command to automatically align at a tag, ends once facing the tag */
public class AlignAtTagWithX extends Command {
	private final SwerveDrivetrain drivetrain;

	private final Vision vision;
	private final int[] tagID;

	private boolean shouldEndImmediately = false;

	private final PIDController xController, yController, rotatePIDtag, rotatePIDangle;

	private double turnToTagSpeed = 0;

	private final double xDistance;

	private final SlewRateLimiter rotateLimiter = new SlewRateLimiter(Units.degreesToRadians(25));

	/**
	 * Create a new AlignAtTag command. Tries to constants Align at a tag while
	 * still
	 * allowing driver to control robot.
	 * 
	 * @param drivetrain the drivetrain of the robot
	 * @param vision     the vision subsystem of the robot
	 * @param tagID      the numerical ID of the tag to turn to, -1 for best tag
	 */
	public AlignAtTagWithX(SwerveDrivetrain drivetrain, Vision vision, int[] tagID, double xDistance,
			Rotation2d rotation) {
		this.drivetrain = drivetrain;

		this.vision = vision;
		this.tagID = tagID;
		this.xDistance = xDistance;

		yController = new PIDController(
				RobotMovementConstants.TRANSLATION_PID_P * 1.25,
				RobotMovementConstants.TRANSLATION_PID_I,
				RobotMovementConstants.TRANSLATION_PID_D);
		yController.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);
		yController.setSetpoint(0);

		xController = new PIDController(
				RobotMovementConstants.TRANSLATION_PID_P,
				RobotMovementConstants.TRANSLATION_PID_I,
				RobotMovementConstants.TRANSLATION_PID_D);
		xController.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);
		xController.setSetpoint(xDistance);

		rotatePIDtag = new PIDController(
				RobotMovementConstants.ROTATION_PID_P,
				RobotMovementConstants.ROTATION_PID_I,
				RobotMovementConstants.ROTATION_PID_D);
		rotatePIDtag.enableContinuousInput(-Math.PI, Math.PI);
		rotatePIDtag.setTolerance(RobotMovementConstants.ANGLE_TOLERANCE_RADIANS);
		rotatePIDtag.setSetpoint(0);

		rotatePIDangle = new PIDController(
				RobotMovementConstants.ROTATION_PID_P,
				RobotMovementConstants.ROTATION_PID_I,
				RobotMovementConstants.ROTATION_PID_D);
		rotatePIDangle.enableContinuousInput(-Math.PI, Math.PI);
		rotatePIDangle.setTolerance(RobotMovementConstants.ANGLE_TOLERANCE_RADIANS);
		rotatePIDangle.setSetpoint(rotation.getRadians());

		addRequirements(drivetrain);
	}

	public AlignAtTagWithX(SwerveDrivetrain drivetrain, Vision vision, int[] tagID, double xDistance) {
		this(drivetrain, vision, tagID, xDistance, new Rotation2d());
	}

	@Override
	public void initialize() {

		// Reset all PID controllers
		rotatePIDangle.reset();
		rotatePIDtag.reset();
		rotatePIDtag.reset();
		yController.reset();

		shouldEndImmediately = true;

		// Check if we see a tag in the tag list, if we do then cancel immediate ending
		for (int tag : tagID) {
			if (vision.getTransformToTag(tag) != null) {
				shouldEndImmediately = false;
				break;
			}
		}
	}

	@Override
	public void execute() {

		// Loop though all tags until we see a transform to a tag
		Transform3d transform = null;
		for (int tag : tagID) {
			transform = vision.getTransformToTag(tag);
			if (transform != null)
				break;
		}

		// Default to not driving
		double xSpeed = 0;
		double ySpeed = 0;

		// Default rotation to whatever the last rotation to the tag is
		// This helps incase we turn away from the tag during driving
		double rotationSpeed = turnToTagSpeed;

		if (transform != null) {

			// If we see the tag update the rotation to the tag
			turnToTagSpeed = transform.getY() > 0 ? 0.01 : -0.01;

			boolean innerPhase = Math.abs(transform.getX()) < 0.5 + xDistance && Math.abs(transform.getY()) < 0.5;

			// Use X and Y controllers to drive to desired distance from tag
			xSpeed = xController.calculate(transform.getX()) * (innerPhase ? 1 : 0.75);
			ySpeed = yController.calculate(transform.getY());

			// Switch to determine when to switch to always facing given angle.
			// If we are far enough away then drive by looking at tag so we don't lose the
			// target
			// If we are close enough then we just lock the angle

			if (innerPhase) {
				rotationSpeed = rotatePIDangle.calculate(drivetrain.getHeading().getRadians());
			} else {
				rotationSpeed = rotatePIDtag.calculate(new Rotation2d(transform.getX(), transform.getY()).unaryMinus().getRotations());
			}
		}

		drivetrain.setDesiredState(new ChassisSpeeds(xSpeed, ySpeed, (rotationSpeed)), false);
	}

	@Override
	public boolean isFinished() {
		return shouldEndImmediately || ((yController.atSetpoint() && xController.atSetpoint())
				&& (rotatePIDtag.atSetpoint() || rotatePIDangle.atSetpoint()));
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}