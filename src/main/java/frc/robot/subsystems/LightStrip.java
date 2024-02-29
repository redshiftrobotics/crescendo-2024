package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightStrip extends SubsystemBase {
	private AddressableLED ledStrip;
	private AddressableLEDBuffer ledBuffer;

	public LightStrip(AddressableLED ledstrip) {
		ledStrip = ledstrip;
		ledBuffer = new AddressableLEDBuffer(60);

		// Docs say this is an expensive operation so future maintainers should avoid
		// modifying this excessibely
		ledStrip.setLength(ledBuffer.getLength());
		ledStrip.setData(ledBuffer);

		ledStrip.start();
	}

	public void setColor(int r, int g, int b) {
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setRGB(i, r, g, b);
		}
	}

	@Override
	public void periodic() {
		ledStrip.setData(ledBuffer);
	}
}
