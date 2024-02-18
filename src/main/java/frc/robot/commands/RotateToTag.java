package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class RotateToTag extends AutoRotateTo {
	// Default
	/**
	 * Rotate to face an AprilTag
	 * 
	 * @param drivetrain the drivetrain subsystem of the robot
	 * @param distToTag  the Transform3d between the robot center and the robot
	 * @param offset     the desired offset angle from the direction of the tag
	 */
	public RotateToTag(SwerveDrivetrain drivetrain, Transform3d distToTag, Rotation2d offset) {
		super(drivetrain, new Rotation2d(distToTag.getX(), distToTag.getY()).plus(offset));
	}

	/**
	 * Rotate to face an AprilTag
	 * 
	 * @param drivetrain the drivetrain subsystem of the robot
	 * @param distToTag  the Transform3d between the robot center and the robot
	 */
	public RotateToTag(SwerveDrivetrain drivetrain, Transform3d distToTag) {
		this(drivetrain, distToTag, new Rotation2d());
	}

	/**
	 * Rotate to face an AprilTag
	 * 
	 * @param drivetrain the drivetrain subsystem of the robot
	 * @param vision     the vision subsystem of the robot
	 * @param tagID      the fiducial ID of the AprilTag to rotateTo
	 */
	public RotateToTag(SwerveDrivetrain drivetrain, Vision vision, int tagID) {
		this(drivetrain, vision.getDistToTag(tagID), new Rotation2d());
	}

	/**
	 * Rotate to face an AprilTag
	 * 
	 * @param drivetrain the drivetrain subsystem of the robot
	 * @param vision     the vision subsystem of the robot
	 * @param tagID      the fiducial ID of the AprilTag to rotateTo
	 * @param offset     the desired offset angle from the direction of the tag
	 */
	public RotateToTag(SwerveDrivetrain drivetrain, Vision vision, int tagID, Rotation2d offset) {
		this(drivetrain, vision.getDistToTag(tagID), offset);
	}

	/**
	 * Rotate to face the best AprilTag in view if found
	 * 
	 * @param drivetrain the drivetrain subsystem of the robot
	 * @param vision     the vision subsystem of the robot
	 */
	public RotateToTag(SwerveDrivetrain drivetrain, Vision vision) {
		this(drivetrain, vision.getDistToTag(), new Rotation2d());
	}

	/**
	 * Rotate to face the best AprilTag in view if found
	 * 
	 * @param drivetrain the drivetrain subsystem of the robot
	 * @param vision     the vision subsystem of the robot
	 * @param offset     the desired offset angle from the direction of the tag
	 */
	public RotateToTag(SwerveDrivetrain drivetrain, Vision vision, Rotation2d offset) {
		this(drivetrain, vision.getDistToTag(), offset);
	}
}
