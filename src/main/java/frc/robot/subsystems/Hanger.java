package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangerConstants;


public class Hanger extends SubsystemBase {
    
    private final CANSparkMax hangerMotorL;
    private final CANSparkMax hangerMotorR;
    private final CANcoder hangerEncoderL;
    private final CANcoder hangerEncoderR;
    private final PIDController hangerPIDController;
    private final StatusSignal<Double> hangerPositionL;
    private final StatusSignal<Double> hangerPositionR;
    private final DigitalInput magnet;
    
    public Hanger(int hangerMotorLID, int hangerMotorRID, int hangerEncoderLID, int hangerEncoderRID, int HANGER_PID_P, int HANGER_PID_I, int HANGER_PID_D) {

        hangerMotorL = new CANSparkMax(hangerMotorLID, MotorType.kBrushless);
        hangerMotorR = new CANSparkMax(hangerMotorRID, MotorType.kBrushless);
        hangerEncoderL = new CANcoder(hangerEncoderLID);
        hangerEncoderR = new CANcoder(hangerEncoderRID);
        DigitalInput magnet = new DigitalInput(0);

        hangerPIDController = new PIDController(HANGER_PID_P, HANGER_PID_I, HANGER_PID_D);

        hangerPositionL = hangerEncoderL.getAbsolutePosition();
        hangerPositionR = hangerEncoderR.getAbsolutePosition();
    }

    public void extendRope() {
        hangerMotorL.set(HangerConstants.HANGER_MOTOR_N);
    }

    public void intakeRope() {
        if (magnet.get()) {
            hangerMotorL.set(0);
            hangerMotorR.set(0);
        }
        else {
            hangerMotorL.set(HangerConstants.HANGER_MOTOR_P);
            hangerMotorR.set(HangerConstants.HANGER_MOTOR_P);
        }
    }

    @Override
    public void periodic() {        
    }
    
}
