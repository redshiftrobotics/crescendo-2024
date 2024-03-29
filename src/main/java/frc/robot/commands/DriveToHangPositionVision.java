package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

public class DriveToHangPositionVision extends Command {
	private final SwerveDrivetrain drivetrain;
	private final Vision vision;

	private final double tagOffset = 0.5; // Meters
	private final int[] useableTags = { 11, 12, 13, 14, 15, 16 };

	public DriveToHangPositionVision(SwerveDrivetrain drivetrain, Vision vision) {
		this.drivetrain = drivetrain;
		this.vision = vision;
		addRequirements(drivetrain, vision);
	}

	@Override
	public void initialize() {
		if (vision != null && vision.isEnabled()) {
			Transform3d tagPos = null;
			for (int i : useableTags) {
				Transform3d newTagPos = vision.getTransformToTag(i);
				if (newTagPos != null) {
					tagPos = newTagPos;
				}

			}

			if (tagPos != null) {
				Rotation2d rotateGoal = tagPos.getRotation().toRotation2d();
				Translation2d transGoal = tagPos.getTranslation().toTranslation2d()
						.plus(new Translation2d(-tagOffset, 0))
						.rotateBy(rotateGoal.unaryMinus());

				Command drive = Commands.sequence(new AutoRotateTo(drivetrain, rotateGoal, false),
						new AutoDriveTo(drivetrain, transGoal));
				drive.schedule();
			}

		}

	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
