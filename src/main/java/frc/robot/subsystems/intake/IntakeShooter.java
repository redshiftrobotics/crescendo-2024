package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IntakeShooter extends SubsystemBase {
	public abstract void setFlyWheelShooterSpeed(double speed);

	public abstract void setIntakeGrabberSpeed(double speed);

	public abstract boolean hasNoteInIntake();

	public abstract void eject();

	public abstract void stop();
}
