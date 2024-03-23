package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeShooterConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

/**
 * The intake & shooter system (mounted to the end of the arm)
 *
 * Other proposed names for this class were:
 * The Relocator
 * Non-Linear Accelarator
 * I. C. E.
 * Throngler
 * 
 * @author Aceius E.
 */
public class RealShooter extends IntakeShooter {
	/*
	 * 2 motors for flywheel
	 * 2 for intake
	 * neo550s, 2 sim motors,
	 */

	private final WPI_VictorSPX flywheel1;
	private final WPI_VictorSPX flywheel2;

	private final CANSparkMax intake;

	// private final DigitalInput intakeSwitch;

	// https://www.chiefdelphi.com/t/pwf-time-of-flight-help/454813
	// https://www.playingwithfusion.com/frc/2022/javadoc/com/playingwithfusion/TimeOfFlight.html
	private final TimeOfFlight intakeSensor;
	// private final Debouncer sensorDebouncer = new Debouncer(0.02, DebounceType.kRising);
	private final MedianFilter sensorMedianFilter = new MedianFilter(25);

	public RealShooter(int flywheel1Id, int flywheel2Id, int intakeID, int intakeSensorId) {
		this.flywheel1 = new WPI_VictorSPX(flywheel1Id);
		this.flywheel2 = new WPI_VictorSPX(flywheel2Id);

		intakeSensor = new TimeOfFlight(intakeSensorId);
		intakeSensor.setRangingMode(RangingMode.Short, 25);

		// this.intakeSwitch = new DigitalInput(intakeSensorId);

		this.intake = new CANSparkMax(intakeID, CANSparkLowLevel.MotorType.kBrushless);
		intake.setInverted(IntakeShooterConstants.INTAKE_REVERSE);
	}

	@Override
	public void setFlyWheelShooterSpeed(double speed) {
		SmartDashboard.putNumber("FlywheelShooter", speed);

		flywheel1.set(-speed);
		flywheel2.set(-speed);

	}

	@Override
	public void setIntakeGrabberSpeed(double speed) {
		SmartDashboard.putNumber("IntakeGrabber", speed);
		intake.set(speed);
	}

	@Override
	public boolean hasNoteInIntake() {
		// return intakeSwitch.get();

		final double distanceMeters = sensorMedianFilter.calculate(
			intakeSensor.getRange() / 1000
		);

		return distanceMeters < Units.inchesToMeters(3);
	}

	@Override
	public void eject() {
		setIntakeGrabberSpeed(-1);
		setFlyWheelShooterSpeed(-0.1);
	}

	@Override
	public void stop() {
		setFlyWheelShooterSpeed(0);
		setIntakeGrabberSpeed(0);
		flywheel1.stopMotor();
		flywheel2.stopMotor();
		intake.stopMotor();
	}
}
