package frc.robot.subsystems.hang;

public class DummyHang extends Hang {

	@Override
	public void setLeftSpeed(double speed) {
	}

	@Override
	public void setRightSpeed(double speed) {
	}

	@Override
	public boolean isAtBottomLeft() {
		return false;
	}

	@Override
	public boolean isAtBottomRight() {
		return false;
	}
}
