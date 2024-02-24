package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LED {

	private AddressableLED m_led;
	private AddressableLEDBuffer m_ledBuffer;

	public void robotInit() {
		// 9 is the PWM port
		m_led = new AddressableLED(9);

		// Default length of 60 starting with empty output
		m_ledBuffer = new AddressableLEDBuffer(60);
		m_led.setLength(m_ledBuffer.getLength());

		// seting the data
		m_led.setData(m_ledBuffer.getLength());
		m_led.start();

		for (int i = 0; i < m_ledBuffer.getLength(); i++) {
			// looping between red and black
			while (true) {
				m_ledBuffer.setHSV(i, 1, 100, 64);
				wait(5);
				m_ledBuffer.setHSV(i, 0, 0, 0);
				wait(5);
			}
		}

		m_led.setData(m_ledBuffer);
	}
}
