package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class RotateToTag extends AutoRotateTo {
	// Default
	public RotateToTag(SwerveDrivetrain drivetrain, Transform3d distToTag, Rotation2d offset) {
		super(drivetrain, new Rotation2d(distToTag.getX(), distToTag.getY()).plus(offset));
	}

	public RotateToTag(SwerveDrivetrain drivetrain, Transform3d distToTag) {
		this(drivetrain, distToTag, new Rotation2d());
	}

	public RotateToTag(SwerveDrivetrain drivetrain, Vision vision, int tagID) {
		this(drivetrain, vision.getDistToTag(tagID), new Rotation2d());
	}

	public RotateToTag(SwerveDrivetrain drivetrain, Vision vision, int tagID, Rotation2d offset) {
		this(drivetrain, vision.getDistToTag(tagID), offset);
	}

	public RotateToTag(SwerveDrivetrain drivetrain, Vision vision) {
		this(drivetrain, vision.getDistToTag(), new Rotation2d());
	}

	public RotateToTag(SwerveDrivetrain drivetrain, Vision vision, Rotation2d offset) {
		this(drivetrain, vision.getDistToTag(), offset);
	}
}
