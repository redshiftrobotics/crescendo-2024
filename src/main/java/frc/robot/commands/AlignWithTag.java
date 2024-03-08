package frc.robot.commands;

import frc.robot.Constants.RobotMovementConstants;
import frc.robot.inputs.ChassisDriveInputs;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/** Command to automatically align at a tag, ends once facing the tag */
public class AlignWithTag extends Command {
	private final SwerveDrivetrain drivetrain;
	private final ChassisDriveInputs chassisDriveInputs;

	private final Vision vision;
	private final int tagID;

	private final PIDController yController;

	/**
	 * Create a new AlignWithTag command. Tries to constants aim at a tag while still
	 * allowing driver to control robot.
	 * 
	 * @param drivetrain          the drivetrain of the robot
	 * @param vision              the vision subsystem of the robot
	 * @param tagID               the numerical ID of the tag to align with, -1 for best tag
	 */
	public AlignWithTag(SwerveDrivetrain drivetrain, Vision vision, int tagID, ChassisDriveInputs chassisDriveInputs) {
		this.drivetrain = drivetrain;

		this.vision = vision;
		this.tagID = tagID;
		this.chassisDriveInputs = chassisDriveInputs;

		yController = new PIDController(1, 0, 0);
		yController.setTolerance(RobotMovementConstants.POSITION_TOLERANCE_METERS);
		yController.setSetpoint(0);

		addRequirements(drivetrain);
	}

	/**
	 * Create a new AlignWithTag command. Tries to aim at a tag.
	 * 
	 * @param drivetrain the drivetrain of the robot
	 * @param vision     the vision subsystem of the robot
	 * @param tagID      the numerical ID of the tag to align with, null for best
	 *                   tag
	 */
	public AlignWithTag(SwerveDrivetrain drivetrain, Vision vision, Integer tagID) {
		this(drivetrain, vision, tagID, null);
	}

	@Override
	public void initialize() {
		drivetrain.toDefaultStates();
		yController.reset();
	}

	@Override
	public void execute() {
		Transform3d transform = vision.getTransformToTag(tagID);

		double tagYawRadians = 0;
		if (transform != null) {
			Rotation2d angleToTag = new Rotation2d(transform.getX(), transform.getY());
			tagYawRadians = angleToTag.getRadians();
		}

		double forwardSpeed = 0;
		double rotationSpeed = 0;
		if (chassisDriveInputs != null) {
			forwardSpeed = chassisDriveInputs.getX();
			rotationSpeed = chassisDriveInputs.getRotation();
		}
		double leftSpeed = yController.calculate(tagYawRadians);

		ChassisSpeeds desiredSpeeds = new ChassisSpeeds(forwardSpeed, leftSpeed, rotationSpeed);

		drivetrain.setDesiredState(desiredSpeeds, false, true);

		drivetrain.updateSmartDashboard();
	}

	@Override
	public boolean isFinished() {
		return (chassisDriveInputs == null) && (yController.atSetpoint());
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}