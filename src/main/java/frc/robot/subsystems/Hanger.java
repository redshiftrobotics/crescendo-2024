package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Hanger extends SubsystemBase {
    
    private final CANSparkMax hangerMotor;
    private final CANcoder hangerEncoder;
    private final PIDController hangerPIDController;
    private final StatusSignal<Double> hangerPosition;
    private final Joystick joystick;
    
    public Hanger(int hangerMotorID, int hangerEncoderID, int HANGER_PID_P, int HANGER_PID_I, int HANGER_PID_D) {

        hangerMotor = new CANSparkMax(hangerMotorID, MotorType.kBrushless);
        hangerEncoder = new CANcoder(hangerEncoderID);

        hangerPIDController = new PIDController(HANGER_PID_P, HANGER_PID_I, HANGER_PID_D);

        hangerPosition = hangerEncoder.getAbsolutePosition();

        joystick = new Joystick(OPERATOR_JOYSTICK_PORT);
    }

    @Override
    public void periodic() {
        whileTrue()
    }
    
}
