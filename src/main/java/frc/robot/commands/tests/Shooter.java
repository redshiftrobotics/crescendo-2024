package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooter;

public class Shooter extends Command {
    private final IntakeShooter intakeShooter;
    private final int speed;
    private final long endSpinning;

    public Shooter(IntakeShooter intakeShooter, int speed, long spinTimeMillis) {
        this.intakeShooter = intakeShooter;
        addRequirements(this.intakeShooter);

        this.speed = speed;

        long beganSpinning = System.currentTimeMillis();
        this.endSpinning = (beganSpinning + spinTimeMillis);
    }

    @Override
    public void initialize() {
        this.intakeShooter.setFlywheelSpeed(speed);
    }

    @Override
    public boolean isFinished() {
        // Wait 1 second before finishing
        return endSpinning >= System.currentTimeMillis();
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeShooter.stopFlywheels();
    }
}
