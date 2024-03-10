package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a dummy stand in class that represents the arm, in the event that the
 * robot does not have an arm in order to prevent errors.
 */
public class DummyArm extends Arm {

	private Rotation2d armPosition = new Rotation2d();

	@Override
	public void setSetpoint(double degrees) {
		SmartDashboard.putNumber("Arm SP Deg", degrees);
		SmartDashboard.putNumber("Arm Deg", degrees);

		armPosition = Rotation2d.fromDegrees(degrees);
	}

	@Override
	public void setSetpoint(double rotation, double toleranceAngle) {
		setSetpoint(rotation);
	}

	
	@Override
	public void setSetpoint(Rotation2d rotation) {
		setSetpoint(rotation.getDegrees());
	}

	@Override
	public void setSetpoint(Rotation2d rotation, double toleranceAngle) {
		setSetpoint(rotation);
	}

	@Override
	public Rotation2d getArmPosition() {
		return armPosition;
	}

	@Override
	public boolean isAtDesiredPosition() {
		return true;
	}
}