package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class LedSubsystem extends SubsystemBase {

    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    int length;

    public static double ledCount = 0;
    public static final double LED_SPEED = 1;
    public final int stripLength;

    public int[] rgb = new int[3];

    private int[] GREEN_CODE = {0, 255, 0};
    private int[] PURPLE_CODE = {255, 0, 255};
    private int[] YELLOW_CODE = {255, 255, 0};

    public LedSubsystem(int LEDPort, int length) {
        this.length = length;
        this.stripLength = (int) (length / 2);
        m_led = new AddressableLED(LEDPort);
        m_ledBuffer = new AddressableLEDBuffer(length);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void fillRange(int first, int last, int[] color) {
        for (int i = first; i < m_ledBuffer.getLength() && i < last; i++) {
            m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
        }
    }

    public void setColor(int c) {
        if (c == 0) {
            rgb = PURPLE_CODE;
        }
        if (c == 1) {
            rgb = YELLOW_CODE;
        }
    }

    public void setLed(int i, int[] color) {
        if (i < 120 && i >= 0) {
            m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
        }
    }

    /** Method to turn on a certain percentage of the lights of a LED strip to a certain color. */
    public void turnOn(double portionLED, int r, int g, int b, int r2, int g2, int b2) {
        assert portionLED <= 1;
        for (var i = 0; i <= m_ledBuffer.getLength(); i++) {
            if (i <= m_ledBuffer.getLength() * portionLED) {
                m_ledBuffer.setRGB(i, r, g, b);
            } else {
                m_ledBuffer.setRGB(i, r2, g2, b2);
            }
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

    public void turnOff() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        m_led.setData(m_ledBuffer);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            setLed(i, GREEN_CODE);
        }

        if (RobotContainer.getIsCone()) setColor(1);
        else setColor(0);
        ledCount += LED_SPEED;
        for (int i = 0; i < stripLength; i++) {
            setLed(((int) (i + ledCount) % length), rgb);
        }

        if (ledCount > 120) {
            ledCount = 0;
        }

        m_led.setData(m_ledBuffer);
        m_led.start();
    }
}
