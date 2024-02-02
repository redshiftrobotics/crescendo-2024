package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Hanger extends SubsystemBase {
    
    private final CANSparkMax hangerMotor;
    private final CANcoder hangerEncoder;
    int hangerMotorID;
    int hangerEncoderID;
    
    public Hanger() {
        hangerMotor = new CANSparkMax(hangerMotorID, MotorType.kBrushless);
        hangerEncoder = new CANcoder(hangerEncoderID);
    }

    @Override
    public void periodic() {

    }
    
}
