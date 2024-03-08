package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Hang extends SubsystemBase {
	public abstract void setLeftSpeed(double speed);

	public abstract void setRightSpeed(double speed);

	public abstract boolean isAtBottomRight();

	public abstract boolean isAtBottomLeft();
}
