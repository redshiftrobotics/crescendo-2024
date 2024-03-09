package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hang.Hang;

public class SetHangSpeed extends Command {

	private final Hang hang;
	private final double speed;
	private final int counterMax = 15;

	private int counter = 0;

	public SetHangSpeed(Hang hang, double speed) {
		this.hang = hang;
		this.speed = speed;
	}

	@Override
	public void initialize() {
		counter = 0;
	}

	@Override
	public void execute() {
		// hang.setRightSpeed(speed);
		// hang.setLeftSpeed(speed);
		if (hang.isAtBottomLeft() && counter > counterMax) {
			hang.setLeftSpeed(0);
		} else {
			hang.setLeftSpeed(speed);
		}

		if (hang.isAtBottomRight() && counter > counterMax) {
			hang.setRightSpeed(0);
		} else {
			hang.setRightSpeed(speed);
		}
	}

	@Override
	public boolean isFinished() {
		return (counter > counterMax) && (hang.isAtBottomLeft() && hang.isAtBottomRight());
	}

	@Override
	public void end(boolean interrupted) {
		hang.setLeftSpeed(0);
		hang.setRightSpeed(0);
	}
}
