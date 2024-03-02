package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoPosition extends Command {
	private final SwerveDrivetrain drivetrain;
	private final Vision vision;

	public AutoPosition(SwerveDrivetrain drivetrain, Vision vision) {
		this.drivetrain = drivetrain;
		this.vision = vision;

		addRequirements(this.drivetrain);
		addRequirements(this.vision); // not sure if we need this?
	}

	@Override
	public void initialize() {
		drivetrain.toDefaultStates();
		Transform3d dist3d = vision.getTransformToTag(1);
		if (dist3d == null) {
			return;
		}

		double angle = dist3d.getRotation().getZ();
		Translation2d trans = new Translation2d(
				dist3d.getX() - 0.5, dist3d.getY());
		SmartDashboard.putNumber("ANGLE", angle);
		SmartDashboard.putNumber("tr X", trans.getX());
		SmartDashboard.putNumber("tr Y", trans.getY());
		Rotation2d rot = new Rotation2d(-(Math.PI - angle));
		Commands.sequence(
				new AutoRotateTo(drivetrain, rot, false), new AutoDriveTo(drivetrain, trans)).schedule();
		;

	}

	@Override
	public void execute() {
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end(boolean interrupted) {
	}
}
