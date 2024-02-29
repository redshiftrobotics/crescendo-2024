package frc.robot.subsystems;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
public class IntakeShooter extends SubsystemBase {
    /*
     * 2 motors for flywheel
     * 2 for intake
     * neo550s, 2 sim motors,
     */

    private final Talon flywheel1; // AndyMark CIM, doublecheck this
    private final Talon flywheel2;
    private final CANSparkMax intake1; 
    private final CANSparkMax intake2; // might not exist

    public IntakeShooter(int flywheel1id, int flywheel2id, int intake1id, int intake2id) {
        this.flywheel1 = new Talon(flywheel1id);
        this.flywheel2 = new Talon(flywheel2id);
        this.intake1 = new CANSparkMax(intake1id, CANSparkLowLevel.MotorType.kBrushless);
        this.intake2 = new CANSparkMax(intake2id, CANSparkLowLevel.MotorType.kBrushless);
    }

    public void setFlywheelSpeed(double speed) {
        flywheel1.set(speed);
        flywheel2.set(speed);
    }

    public void stopFlywheels() {
        flywheel1.stopMotor();
        flywheel2.stopMotor();
    }

    /**
     * This method is called periodically by the CommandScheduler, about every 20ms.
     * It should be used for updating subsystem-specific state that you don't want to offload to a Command.
     * Try to avoid "doing to much" in this method (for example no driver control here).
     */
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
