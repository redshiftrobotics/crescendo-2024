package frc.robot.subsystems.hang;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

public class RealHang extends Hang {

	private final CANSparkMax leftMotor;
	private final CANSparkMax rightMotor;

	private final DigitalInput limitSwitch;

	private double speed;

	public RealHang(int leftMotorID, int rightMotorID, boolean leftMotorIsInverted, boolean rightMotorIsInverted,
			int limitSwitchId) {
		leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
		rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);

		limitSwitch = new DigitalInput(limitSwitchId);

		leftMotor.setInverted(leftMotorIsInverted);
		rightMotor.setInverted(rightMotorIsInverted);
	}

	@Override
	public void periodic() {
		leftMotor.set(speed);
		rightMotor.set(speed);
	}

	public void setSpeed(double speed) {
		this.speed = speed;
	}

	public boolean isAtBottom() {
		return limitSwitch.get();
	}
}
