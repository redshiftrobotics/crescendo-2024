package frc.robot.subsystems.hang;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Hang extends SubsystemBase {
	public abstract void setSpeed(double speed);

	public abstract boolean isAtBottom();
}
