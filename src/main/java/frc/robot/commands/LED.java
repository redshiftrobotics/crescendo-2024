package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.Command;

public class LED extends Command {

	private AddressableLED m_led;
	private AddressableLEDBuffer m_ledBuffer;
	private int h = 0;
	private int s = 0;
	private int v = 0;

	private void setColor(int h, int s, int v) {
		for (int i = 0; i < m_ledBuffer.getLength(); i++) {
			m_ledBuffer.setHSV(i, h, s, v);
		}
	}

	public LED(AddressableLED m_led, AddressableLEDBuffer m_ledBuffer, int h, int s, int v) {
		this.m_led = m_led;
		this.m_ledBuffer = m_ledBuffer;
		this.h = h;
		this.s = s;
		this.v = v;
	}

	@Override
	public void initialize() {
		// 9 is the PWM port
		// m_led = new AddressableLED(9);
		// Default length of 60 starting with empty output
		// m_ledBuffer = new AddressableLEDBuffer(60);

		m_led.setLength(m_ledBuffer.getLength());

		// seting the data
		m_led.setData(m_ledBuffer);
		m_led.start();

		setColor(h, s, v);

		m_led.setData(m_ledBuffer);
	}

	@Override
	public boolean isFinished() { return true; }

}
