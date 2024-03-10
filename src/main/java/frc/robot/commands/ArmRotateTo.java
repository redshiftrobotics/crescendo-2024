package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class ArmRotateTo extends Command {

	private final Rotation2d setpoint;
	private final Arm arm;
	private final double toleranceAngle;

	public ArmRotateTo(Arm arm, Rotation2d rotation, double toleranceAngle) {
		this.setpoint = rotation;
		this.arm = arm;
		this.toleranceAngle = toleranceAngle;
		
		addRequirements(arm);
	}

	public ArmRotateTo(Arm arm, Rotation2d rotation) {
		this(arm, rotation, 2);
	}

	public ArmRotateTo(Arm arm, double degrees) {
		this(arm, Rotation2d.fromDegrees(degrees), 2);
	}

	public ArmRotateTo(Arm arm, double degrees, double toleranceAngle) {
		this(arm, Rotation2d.fromDegrees(degrees), toleranceAngle);
	}

	@Override
	public void initialize() {
		arm.setSetpoint(setpoint, toleranceAngle);
	}

	@Override
	public boolean isFinished() {
		return arm.isAtDesiredPosition();
	}
}
