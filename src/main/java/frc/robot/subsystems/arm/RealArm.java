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
		

		armPosition = rightArmEncoder.getAbsolutePosition();

		leftArmMotor.setIdleMode(IdleMode.kCoast);
		rightArmMotor.setIdleMode(IdleMode.kCoast);

		leftArmMotor.setInverted(areMotorsReversed);
		rightArmMotor.setInverted(!areMotorsReversed);

		setSetpoint(ArmConstants.ARM_INTAKE_DEGREES);
	}

	public void setSetpoint(double degrees) {
		degrees = Math.max(degrees, ArmConstants.MINIMUM_ARM_DEGREES);
		degrees = Math.min(degrees, ArmConstants.MAXIMUM_ARM_DEGREES);

		SmartDashboard.putNumber("Arm Setpoint Degrees", degrees);

		armRaisePIDController.setSetpoint(Units.degreesToRotations(degrees));
	}

	public Rotation2d getArmPosition() {
		return Rotation2d.fromRotations(armPosition.refresh().getValueAsDouble());
	}

	public boolean isAtDesiredPosition() {
		return armRaisePIDController.atSetpoint();
	}

	/**
	 * This method is called periodically by the CommandScheduler, about every 20ms.
	 */
	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		Rotation2d currentOnPosition = getArmPosition();

		double armSpeed = armRaisePIDController.calculate(currentOnPosition.getRotations());

		leftArmMotor.set(armSpeed);
		rightArmMotor.set(armSpeed);

		SmartDashboard.putNumber("Arm Speed", armSpeed);
		SmartDashboard.putNumber("Arm Degrees", currentOnPosition.getDegrees());
	}
}
