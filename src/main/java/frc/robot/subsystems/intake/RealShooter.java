package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeShooterConstants;

/**
 * The intake & shooter system (mounted to the end of the arm)
 *
 * Other proposed names for this class were:
 * The Relocator
 * Non-Linear Accelarator
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

	public RealShooter(int flywheel1Id, int flywheel2Id, int intakeID, int intakeLimitSwitchId, int lidarId) {
		this.flywheel1 = new WPI_VictorSPX(flywheel1Id);
		this.flywheel2 = new WPI_VictorSPX(flywheel2Id);

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
		return false;
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

	public double getLidar() {
		return 0;
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Lidar", getLidar());
	}
}
