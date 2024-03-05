package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;

public class DummyShooter extends IntakeShooter {

	private long flywheelSpinUpTime;

	public DummyShooter() {
		flywheelSpinUpTime = System.currentTimeMillis();
	}

	@Override
	public void setFlyWheelShooterSpeed(double speed) {
		flywheelSpinUpTime = System.currentTimeMillis();
	}

	@Override
	public double getFlyWheelShooterSpinUpTimeSeconds() {
		return Units.millisecondsToSeconds(flywheelSpinUpTime);
	}

	@Override
	public void setIntakeGrabberSpeed(double speed) {
	}

	@Override
	public void eject() {
	}

	@Override
	public void stop() {
	}
}
