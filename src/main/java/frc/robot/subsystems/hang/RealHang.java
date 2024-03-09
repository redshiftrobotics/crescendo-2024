package frc.robot.subsystems.hang;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

public class RealHang extends Hang {

	private final CANSparkMax motor;

	private final DigitalInput leftLimitSwitch;

	public RealHang(int motorID, boolean motorIsInverted, int rightLimitSwitchId) {
		motor = new CANSparkMax(motorID, MotorType.kBrushless);
		motor.setInverted(motorIsInverted);
		motor.setIdleMode(IdleMode.kBrake);

		leftLimitSwitch = new DigitalInput(rightLimitSwitchId);
	}


	@Override
	public void setSpeed(double speed) {
		motor.set(speed);
	}

	@Override
	public boolean isAtBottom() {
		return leftLimitSwitch.get();
	}
}
