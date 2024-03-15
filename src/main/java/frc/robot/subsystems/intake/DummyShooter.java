package frc.robot.subsystems.intake;

public class DummyShooter extends IntakeShooter {

	public DummyShooter() {
	}

	@Override
	public void setFlyWheelShooterSpeed(double speed) {
	}

	@Override
	public void setIntakeGrabberSpeed(double speed) {
	}

	@Override
	public double howLongAtSpeedMillis(double speed) {
		return 0;
	}

	@Override
	public boolean hasNoteInIntake() {
		return false;
	}

	@Override
	public void eject() {
	}

	@Override
	public void stop() {
	}
}
