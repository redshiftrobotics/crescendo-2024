package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotMovementConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoRotateTo extends Command {
	private final SwerveDrivetrain drivetrain;

	private final PIDController rotatePID;
	private final double angleGoal;
	private final boolean fieldRelative;

	private double currentAngleGoal;

	/***
	 * Command to autonomously rotate some direction
	 * 
	 * @param drivetrain The robot drivetrain
	 * @param direction  Rotation2d class to execute
	 */
	public AutoRotateTo(SwerveDrivetrain drivetrain, Rotation2d direction, boolean fieldRelative) {

		rotatePID = new PIDController(
				RobotMovementConstants.ROTATION_PID_P,
				RobotMovementConstants.ROTATION_PID_I,
				RobotMovementConstants.ROTATION_PID_D);
		rotatePID.enableContinuousInput(-Math.PI, Math.PI);
		rotatePID.setTolerance(RobotMovementConstants.ANGLE_TOLERANCE_RADIANS);

		this.drivetrain = drivetrain;
		this.fieldRelative = fieldRelative;
		this.angleGoal = direction.getRadians();

		addRequirements(this.drivetrain);
	}

	public AutoRotateTo(SwerveDrivetrain drivetrain, Rotation2d direction) {
		this(drivetrain, direction, true);
	}

	@Override
	public void initialize() {
		currentAngleGoal = fieldRelative ? 0 : drivetrain.getHeading().getRadians();
		currentAngleGoal += angleGoal;
		// SmartDashboard.putNumber("Target Angle Auto", currentAngleGoal);
	}

	@Override
	public void execute() {
		final double currentAngle = drivetrain.getHeading().getRadians();

		double turnsSeed = rotatePID.calculate(currentAngle, this.currentAngleGoal);

		drivetrain.setDesiredState(new ChassisSpeeds(0, 0, (turnsSeed)));
	}

	@Override
	public boolean isFinished() {
		return rotatePID.atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}
