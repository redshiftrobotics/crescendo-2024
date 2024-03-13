package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;

/** This is the subsystem that represents the arm. */
public class RealArm extends Arm {

	private final CANSparkMax leftArmMotor;
	private final CANSparkMax rightArmMotor;

	private final CANcoder rightArmEncoder;

	private final StatusSignal<Double> armPosition;

	private final PIDController armRaisePIDController;

	/** Constructor. Creates a new Arm Subsystem. */
	public RealArm(int leftMotorId, int rightMotorId, int rightEncoderId, boolean areMotorsReversed) {

		leftArmMotor = new CANSparkMax(leftMotorId, MotorType.kBrushless);
		rightArmMotor = new CANSparkMax(rightMotorId, MotorType.kBrushless);

		rightArmEncoder = new CANcoder(rightEncoderId);

		armRaisePIDController = new PIDController(
				ArmConstants.ELEVATION_PID_P,
				ArmConstants.ELEVATION_PID_I,
				ArmConstants.ELEVATION_PID_D);
		armRaisePIDController.setSetpoint(ArmConstants.ARM_START_DEGREES);

		armPosition = rightArmEncoder.getAbsolutePosition();

		leftArmMotor.setIdleMode(IdleMode.kBrake);
		rightArmMotor.setIdleMode(IdleMode.kBrake);

		leftArmMotor.setInverted(areMotorsReversed);
		rightArmMotor.setInverted(!areMotorsReversed);
	}

	@Override
	public void setSetpoint(double degrees) {
		setSetpoint(degrees, 2);
	}

	@Override
	public void setSetpoint(double degrees, double toleranceAngle) {
		degrees = Math.max(degrees, ArmConstants.MINIMUM_ARM_DEGREES);
		degrees = Math.min(degrees, ArmConstants.MAXIMUM_ARM_DEGREES);
		SmartDashboard.putNumber("Arm SP Deg", degrees);

		armRaisePIDController.setSetpoint(Units.degreesToRotations(degrees));
	}

	@Override
	public void setSetpoint(Rotation2d rotation) {
		setSetpoint(rotation, Units.degreesToRotations(2));
	}

	@Override
	public void setSetpoint(Rotation2d rotation, double toleranceAngle) {
		setSetpoint(rotation.getDegrees());
	}

	@Override
	public Rotation2d getArmPosition() {
		return Rotation2d.fromRotations(armPosition.refresh().getValueAsDouble());
	}

	@Override
	public boolean isAtDesiredPosition() {
		return armRaisePIDController.atSetpoint();
	}

	@Override
	public void periodic() {
		Rotation2d currentOnPosition = getArmPosition();

		double armSpeed = armRaisePIDController.calculate(currentOnPosition.getRotations());

		leftArmMotor.set(armSpeed);
		rightArmMotor.set(armSpeed);

		SmartDashboard.putNumber("Arm Deg", currentOnPosition.getDegrees());
	}
}
