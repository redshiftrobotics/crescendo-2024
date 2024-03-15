package frc.robot.commands;

import frc.robot.Constants.RobotMovementConstants;
import frc.robot.subsystems.ChassisDriveInputs;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/** Command to automatically aim at a tag, ends once facing the tag */
public class AimAtTag extends Command {
	private final SwerveDrivetrain drivetrain;
	private final ChassisDriveInputs chassisDriveInputs;

	private final Vision vision;
	private final int tagID;

	private final PIDController rotatePID;

	/**
	 * Create a new AimAtTag command. Tries to constants aim at a tag while still
	 * allowing driver to control robot.
	 * 
	 * @param drivetrain          the drivetrain of the robot
	 * @param vision              the vision subsystem of the robot
	 * @param tagID               the numerical ID of the tag to turn to, -1 for best tag
	 */
	public AimAtTag(SwerveDrivetrain drivetrain, Vision vision, int tagID, ChassisDriveInputs chassisDriveInputs) {
		this.drivetrain = drivetrain;

		this.vision = vision;
		this.tagID = tagID;
		this.chassisDriveInputs = chassisDriveInputs;

		rotatePID = new PIDController(
				RobotMovementConstants.ROTATION_PID_P,
				RobotMovementConstants.ROTATION_PID_I,
				RobotMovementConstants.ROTATION_PID_D);
		rotatePID.enableContinuousInput(-Math.PI, Math.PI);
		rotatePID.setTolerance(RobotMovementConstants.ANGLE_TOLERANCE_RADIANS);
		rotatePID.setSetpoint(0);

		addRequirements(drivetrain, chassisDriveInputs);
	}

	/**
	 * Create a new AimAtTag command. Tries to aim at a tag.
	 * 
	 * @param drivetrain the drivetrain of the robot
	 * @param vision     the vision subsystem of the robot
	 * @param tagID      the numerical ID of the tag to turn to, null for best
	 *                   tag
	 */
	public AimAtTag(SwerveDrivetrain drivetrain, Vision vision, Integer tagID) {
		this(drivetrain, vision, tagID, null);
	}

	@Override
	public void initialize() {
		drivetrain.toDefaultStates();
	}

	@Override
	public void execute() {
		Transform3d transform = vision.getTransformToTag(tagID);

		double rotationSpeed = 0;
		if (transform != null) {
			Rotation2d angleToTag = new Rotation2d(transform.getX(), transform.getY());
			rotationSpeed = -rotatePID.calculate(angleToTag.getRadians());
		}

		double xSpeed = 0;
		double ySpeed = 0;
		boolean fieldRelative = false;
		if (chassisDriveInputs != null) {
			xSpeed = chassisDriveInputs.getX();
			ySpeed = chassisDriveInputs.getY();
			fieldRelative = chassisDriveInputs.isFieldRelative();
		}

		ChassisSpeeds desiredSpeeds = new ChassisSpeeds(xSpeed, ySpeed, 0);
		if (fieldRelative) desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, drivetrain.getHeading());
		desiredSpeeds.omegaRadiansPerSecond = rotationSpeed;

		drivetrain.setDesiredState(desiredSpeeds, false);

		drivetrain.updateSmartDashboard();
	}

	@Override
	public boolean isFinished() {
		return (chassisDriveInputs == null) && (rotatePID.atSetpoint());
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}