package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.IntakeShooter;

public class MoveToTagAndShootSpeaker extends Command {
	private final SwerveDrivetrain drivetrain;
	private final Vision vision;
	private final Arm arm;
	private final IntakeShooter shooter;
	final int[] tags = { 6, 5 };
	final double tagOffset = 0.1; // Meters

	public MoveToTagAndShootSpeaker(SwerveDrivetrain drivetrain, Vision vision, Arm arm, IntakeShooter intakeShooter) {
		this.drivetrain = drivetrain;
		this.vision = vision;
		this.arm = arm;
		this.shooter = intakeShooter;
		addRequirements(drivetrain, vision, arm, shooter);
	}

	@Override
	public void initialize() {
		if (vision != null && vision.isEnabled()) {
			Transform3d trans = null;
			for (int i : tags) {
				Transform3d newTrans = vision.getTransformToTag(i);
				if (newTrans != null) {
					trans = newTrans;
				}
			}

			if (trans != null) {
				Rotation2d rotateGoal = trans.getRotation().toRotation2d();
				Translation2d transGoal = trans.getTranslation().toTranslation2d()
						.plus(new Translation2d(-tagOffset, 0).rotateBy(rotateGoal.unaryMinus()));
				Command command = Commands.sequence(
						new SpinFlywheelShooter(shooter, IntakeShooterConstants.FLYWHEEL_SHOOTER_SPEED_SPEAKER),
						Commands.parallel(Commands.sequence(new AutoRotateTo(drivetrain, rotateGoal),
								new AutoDriveTo(drivetrain,
										transGoal)),
								new WaitCommand(1.0),
								new ArmRotateTo(arm, ArmConstants.ARM_SPEAKER_SHOOTING_DEGREES),
								new SpinIntakeGrabbers(shooter, IntakeShooterConstants.INTAKE_GRABBER_SPEED_SPEAKER),
								new WaitCommand(0.3),
								new SpinFlywheelShooter(shooter, 0.0),
								new SpinIntakeGrabbers(shooter, 0.0),
								new ArmRotateTo(arm, ArmConstants.ARM_STOW_2_DEGREES))
								.onlyIf(() -> vision != null && vision.isEnabled()));
				command.schedule();
			}
		}
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
