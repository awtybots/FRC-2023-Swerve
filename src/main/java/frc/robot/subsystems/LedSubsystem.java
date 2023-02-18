package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem extends SubsystemBase {

    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    int length;

    public LedSubsystem(int length) {
        this.length = length;
        m_led = new AddressableLED(Constants.CustomConstants.LEDPort);
        m_ledBuffer = new AddressableLEDBuffer(length);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    /** Method to turn on a certain percentage of the lights of a LED strip to a certain color. */
    public void turnOn(double portionLED, int r, int g, int b) {
        assert portionLED <= 1;
        for (var i = 0; i <= m_ledBuffer.getLength() * portionLED; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    /**
     * Method to change the amount of lights a LED strip has as green based on the area of the screen
     * that a target takes up in the Limelight Apriltag detection system.
     */
    public void visionTrackingLED(double area) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            if (area > 17) {
                m_ledBuffer.setRGB(i, 0, 200, 0);
            } else if (area * 10 > i) {
                m_ledBuffer.setRGB(i, 0, 0, 200);
            } else {
                m_ledBuffer.setRGB(i, 100, 100, 100);
            }
        }
        m_led.setData(m_ledBuffer);
    }

    public void ledPatterns() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {}
    }

    public void turnOff() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        m_led.setData(m_ledBuffer);
    }
}
