package frc.robot.commands;

import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Constants.RobotMovementConstants;

// How to make Command (ignore image instructions, code is out of date, just look at written general instructions): https://compendium.readthedocs.io/en/latest/tasks/commands/commands.html
// Command based programming: https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html
// Code documentations https://docs.wpilib.org/en/stable/docs/software/commandbased/commands.html 

/** An example command that uses an example subsystem. */
public class AutoRotateTo extends Command {
	private final SwerveDrivetrain drivetrain;

	private final PIDController rotatePID;
	private final double angleGoal;

	private double atSetpointCounter = 0;

	public AutoRotateTo(SwerveDrivetrain subsystem, Rotation2d direction) {

		rotatePID = new PIDController(
				RobotMovementConstants.ROTATION_PID_P,
				RobotMovementConstants.ROTATION_PID_I,
				RobotMovementConstants.ROTATION_PID_D);

		this.drivetrain = subsystem;
		this.angleGoal = direction.getRadians();

		addRequirements(this.drivetrain);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		final double currentAngle = drivetrain.getHeading().getRadians();

		double turnsSeed = rotatePID.calculate(currentAngle, this.angleGoal);

		drivetrain.setDesiredState(new ChassisSpeeds(0, 0, turnsSeed));

		if (Math.abs(currentAngle - this.angleGoal) < RobotMovementConstants.ANGLE_TOLERANCE_RADIANS)
			atSetpointCounter += TimedRobot.kDefaultPeriod;
		else
			atSetpointCounter = 0;
	}

	@Override
	public boolean isFinished() {
		return atSetpointCounter > RobotMovementConstants.ROTATE_AT_SETPOINT_TIME_SECONDS;
	}

	@Override
	public void end(boolean interrupted) {
		drivetrain.stop();
	}
}
