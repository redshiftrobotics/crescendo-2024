package frc.robot.subsystems.hang;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RealHang extends Hang {

	private final CANSparkMax motor;
	// private final DigitalInput leftLimitSwitch;
	private final String name;

	public RealHang(int motorID, boolean motorIsInverted, int rightLimitSwitchId, String name) {
		motor = new CANSparkMax(motorID, MotorType.kBrushless);
		motor.setInverted(motorIsInverted);
		motor.setIdleMode(IdleMode.kBrake);

		// leftLimitSwitch = new DigitalInput(rightLimitSwitchId);

		this.name = name;

		setName(name + "Hang");
	}

	public RealHang(int motorID, boolean motorIsInverted, int rightLimitSwitchId) {
		this(motorID, motorIsInverted, rightLimitSwitchId, String.valueOf(motorID));
	}

	@Override
	public void periodic() {
		SmartDashboard.putString(name + "Hang", motor.get() + "p" + (isAtBottom() ? "Down" : "Up"));
		SmartDashboard.putBoolean(name + "Hang Is Down", isAtBottom());
	}

	@Override
	public void setSpeed(double speed) {
		motor.set(speed);
	}

	@Override
	public boolean isAtBottom() {
		return true;
		// return leftLimitSwitch.get();
	}
}
