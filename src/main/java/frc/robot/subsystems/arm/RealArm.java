package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** This is the subsystem that represents the arm. */
public class RealArm extends Arm {

	private final SparkMax leftArmMotor;
	private final SparkMax rightArmMotor;

	private final CANcoder rightArmEncoder;

	private final StatusSignal<Angle> armPosition;

	private final PIDController armRaisePIDController;

	/** Constructor. Creates a new Arm Subsystem. */
	public RealArm(int leftMotorId, int rightMotorId, int rightEncoderId, boolean areMotorsReversed) {

		leftArmMotor = new SparkMax(leftMotorId, MotorType.kBrushless);
		rightArmMotor = new SparkMax(rightMotorId, MotorType.kBrushless);

		rightArmEncoder = new CANcoder(rightEncoderId);

		armRaisePIDController = new PIDController(
				ArmConstants.ELEVATION_PID_P,
				ArmConstants.ELEVATION_PID_I,
				ArmConstants.ELEVATION_PID_D);
		armRaisePIDController.setTolerance(Units.degreesToRotations(ArmConstants.ARM_TOLERANCE_DEGREES));

		armPosition = rightArmEncoder.getAbsolutePosition();

		MagnetSensorConfigs magnetSensorConfig = new MagnetSensorConfigs();
		magnetSensorConfig.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
		magnetSensorConfig.AbsoluteSensorDiscontinuityPoint = 1;
		magnetSensorConfig.MagnetOffset = 0;
		rightArmEncoder.getConfigurator().apply(magnetSensorConfig);

		SparkMaxConfig leftArmConfig = new SparkMaxConfig();
		leftArmConfig.idleMode(IdleMode.kBrake);
		leftArmConfig.inverted(areMotorsReversed);

		SparkMaxConfig rightArmConfig = new SparkMaxConfig();
		rightArmConfig.idleMode(IdleMode.kBrake);
		rightArmConfig.inverted(!areMotorsReversed);

		leftArmMotor.configure(leftArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		rightArmMotor.configure(rightArmConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void setSetpoint(double degrees) {
		degrees = MathUtil.clamp(degrees, ArmConstants.MINIMUM_ARM_DEGREES, ArmConstants.MAXIMUM_ARM_DEGREES);

		SmartDashboard.putNumber("Arm SP Deg", degrees);

		armRaisePIDController.setSetpoint(Units.degreesToRotations(degrees));
	}

	@Override
	public void setSetpoint(Rotation2d rotation) {
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

		// double armSpeed = 0;
		double armSpeed = armRaisePIDController.calculate(currentOnPosition.getRotations());

		leftArmMotor.set(armSpeed);
		rightArmMotor.set(armSpeed);
		SmartDashboard.putNumber("Arm Speed", armSpeed);

		SmartDashboard.putNumber("Arm Deg", currentOnPosition.getDegrees());
	}
}
