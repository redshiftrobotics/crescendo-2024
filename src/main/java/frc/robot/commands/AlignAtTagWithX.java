package frc.robot.commands;

import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotMovementConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

/** Command to automatically align at a tag, ends once facing the tag */
public class AlignAtTagWithX extends Command {
	private final double INNER_MODE_BOX_X = 0.5;
	private final double INNER_MODE_BOX_Y = 0.5;

	private final double LOST_TAG_SPEED_RADIANS = 0.01;

	private final SwerveDrivetrain drivetrain;

	private final Vision vision;

	private final IntSupplier tag;
	private final Supplier<Rotation2d> rotation;

	private boolean shouldEndImmediately = false;

	private final PIDController xController, yController, rotatePIDtag, rotatePIDangle;

	private double rotationAfterLosingTag = 0;

	private final double xDistance;

	// private final SlewRateLimiter rotateLimiter = new
	// SlewRateLimiter(Units.degreesToRadians(25));

	/**
	 * Create a new AlignAtTag command. Tries to constants Align at a tag while
	 * still
	 * allowing driver to control robot.
	 * 
	 * @param drivetrain the drivetrain of the robot
	 * @param vision     the vision subsystem of the robot
	 * @param tag        the numerical ID of the tag to turn to, -1 for best tag
	 */
	public AlignAtTagWithX(SwerveDrivetrain drivetrain, Vision vision, IntSupplier tag, double xDistance,
			Supplier<Rotation2d> rotation) {
		this.drivetrain = drivetrain;

		this.vision = vision;
		this.xDistance = xDistance;

		this.tag = tag;
		this.rotation = rotation;

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

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {

		// Reset all PID controllers
		rotatePIDangle.reset();
		rotatePIDtag.reset();
		rotatePIDtag.reset();
		yController.reset();

		shouldEndImmediately = vision.getTransformToTag(tag.getAsInt()) == null;
	}

	@Override
	public void execute() {

		// Get transform to tag
		final Transform3d transform = vision.getTransformToTag(tag.getAsInt());

		// Default to not driving
		double xSpeed = 0;
		double ySpeed = 0;

		SmartDashboard.putString("Tag+Rotation", tag.getAsInt() + ": " + rotation.get().getDegrees());

		// Default rotation to whatever the last rotation to the tag is
		// This helps incase we turn away from the tag during driving
		double rotationSpeed = rotationAfterLosingTag;

		if (transform != null) {

			// If we see the tag update the rotation to the tag
			rotationAfterLosingTag = (transform.getY() > 0 ? 1 : -1) * LOST_TAG_SPEED_RADIANS;

			boolean innerPhase = (transform.getX() < INNER_MODE_BOX_X + xDistance)
					&& (Math.abs(transform.getY()) < INNER_MODE_BOX_Y);

			// Use X and Y controllers to drive to desired distance from tag
			xSpeed = xController.calculate(transform.getX(), xDistance);
			ySpeed = yController.calculate(transform.getY(), 0);
			// SmartDashboard.putNumber("X speed a",
			// xController.calculate(transform.getX()));
			// SmartDashboard.putNumber("Y speed a",
			// yController.calculate(transform.getY()));

			// Switch to determine when to switch to always facing given angle.
			// If we are far enough away then drive by looking at tag so we don't lose the
			// target.
			// If we are close enough then we just lock the angle
			if (innerPhase) {
				rotationSpeed = rotatePIDangle.calculate(drivetrain.getHeading().getRadians(),
						rotation.get().getRadians());
			} else {
				rotationSpeed = rotatePIDtag
						.calculate(new Rotation2d(transform.getX(), transform.getY()).unaryMinus().getRotations());
			}
		}

		drivetrain.setDesiredState(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed), false);
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