package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Hanger extends SubsystemBase {
    
    private final CANSparkMax hangerMotor;
    private final CANcoder hangerEncoder;
    private final PIDController hangerPIDController;
    private final StatusSignal<Double> hangerPosition;

    int hangerMotorID;
    int hangerEncoderID;
    int p;
    int i;
    int d;
    
    public Hanger(int hangerMotorID, int hangerEncoderID) {

        hangerMotor = new CANSparkMax(hangerMotorID, MotorType.kBrushless);
        hangerEncoder = new CANcoder(hangerEncoderID);

        hangerPIDController = new PIDController(p, i, d);

        hangerPosition = hangerEncoder.getAbsolutePosition();
    }

    @Override
    public void periodic() {

    }
    
}
