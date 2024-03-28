package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.RobotMovementConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Vision;

/** Command to automatically align at a tag, ends once facing the tag */
public class FollowTag extends Command {
	private final SwerveDrivetrain drivetrain;

	private final Vision vision;

	private final PIDController xController, yController, rotateController;
	private final SlewRateLimiter xSpeedSlewFilter, ySpeedSlewFilter, rotateSpeedSlewFilter;
	private final SlewRateLimiter xPosSlewFilter, yPosSlewFilter, rotatePosSlewFilter;
	private final MedianFilter xPosMeanFilter, yPosMeanFilter, rotatePosMeanFilter;

	private static final double TIME_TO_SPIN = 3 / TimedRobot.kDefaultPeriod;
	private double spinCounter = 0;

	/**
	 * Create a new AlignAtTag command. Tries to constants Align at a tag while
	 * still
	 * allowing driver to control robot.
	 * 
	 * @param drivetrain the drivetrain of the robot
	 * @param vision     the vision subsystem of the robot
	 * @param tagID      the numerical ID of the tag to turn to, -1 for best tag
	 */
	public FollowTag(SwerveDrivetrain drivetrain, Vision vision, double xDistance) {
		this.drivetrain = drivetrain;

		this.vision = vision;

		yController = new PIDController(
				RobotMovementConstants.TRANSLATION_PID_P,
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

		rotateController = new PIDController(
				RobotMovementConstants.ROTATION_PID_P,
				RobotMovementConstants.ROTATION_PID_I,
				RobotMovementConstants.ROTATION_PID_D);
		rotateController.enableContinuousInput(-Math.PI, Math.PI);
		rotateController.setTolerance(RobotMovementConstants.ANGLE_TOLERANCE_RADIANS);
		rotateController.setSetpoint(0);

		xSpeedSlewFilter = new SlewRateLimiter(1);
		ySpeedSlewFilter = new SlewRateLimiter(1);
		rotateSpeedSlewFilter = new SlewRateLimiter(Units.degreesToRadians(10));

		xPosSlewFilter = new SlewRateLimiter(5);
		yPosSlewFilter = new SlewRateLimiter(5);
		rotatePosSlewFilter = new SlewRateLimiter(Units.degreesToRadians(90));

		xPosMeanFilter = new MedianFilter(2);
		yPosMeanFilter = new MedianFilter(2);
		rotatePosMeanFilter = new MedianFilter(2);

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		rotateController.reset();
		xController.reset();
		yController.reset();

		xSpeedSlewFilter.reset(0);
		ySpeedSlewFilter.reset(0);
		rotateSpeedSlewFilter.reset(0);

		xPosSlewFilter.reset(0);
		yPosSlewFilter.reset(0);
		rotatePosSlewFilter.reset(0);

		xPosMeanFilter.reset();
		yPosMeanFilter.reset();
		rotatePosMeanFilter.reset();

		spinCounter = 0;

		drivetrain.toDefaultStates();
	}

	@Override
	public void execute() {
		Transform3d transform = vision.getTransformToTag();

		double xSpeed = 0;
		double ySpeed = 0;
		double rotateSpeed = 0;
		if (transform != null) {
			spinCounter = 0;
			
			double x = transform.getX();
			double y = transform.getY();
			double rotate = new Rotation2d(transform.getX(), transform.getY()).unaryMinus().getRadians();

			// x = xPosMeanFilter.calculate(x);
			// y = yPosMeanFilter.calculate(y);
			// rotate = rotatePosMeanFilter.calculate(rotate);
			
			// x = xPosSlewFilter.calculate(x);
			// y = yPosSlewFilter.calculate(y);
			// rotate = rotatePosSlewFilter.calculate(rotate);

			xSpeed = xController.calculate(x);
			ySpeed = yController.calculate(y);
			rotateSpeed = rotateController.calculate(rotate);
			
			xSpeed = xSpeedSlewFilter.calculate(xSpeed);
			ySpeed = ySpeedSlewFilter.calculate(ySpeed);
			rotateSpeed = rotateSpeedSlewFilter.calculate(rotateSpeed);
		} else {
			spinCounter++;
		}

		if (spinCounter > TIME_TO_SPIN) {
			rotateSpeed = DriverConstants.MAX_ROTATION_SPEED * 0.05;
		}

		drivetrain.setDesiredState(new ChassisSpeeds(xSpeed, ySpeed, rotateSpeed), false);

		drivetrain.updateSmartDashboard();
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}
