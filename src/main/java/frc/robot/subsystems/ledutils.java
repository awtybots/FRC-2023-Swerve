// like a wise man once said: It just works
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ledutils extends SubsystemBase {

    AddressableLED m_led;
    AddressableLED m_led_b;
    AddressableLEDBuffer m_ledBuffer;
    int length;

    public enum patterens_eneum {
        awtybots,
        awtybotsenhanced,
        percentagedefault,
        awtybotsenhancedenhaced,
        cone,
        cube
    }

    boolean primary = true;

    public ledutils(int LEDPort, int length) {
        this.length = length;
        m_led = new AddressableLED(Constants.DefaultConfig.LEDPort);
        m_ledBuffer = new AddressableLEDBuffer(length);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setLED_RGB_PERCENTAGE(double portionLED, int r, int g, int b) {
        if (portionLED > 1) return;
        for (var i = 0; i < m_ledBuffer.getLength() * portionLED; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    public void setLED_RGB_PERCENTAGE_Strip_spicific(
            double portionLED, int r, int g, int b, boolean a_or_c, int rb, int gb, int bb) {
        if (portionLED > 1) return;
        if (a_or_c) {
            for (var i = 0; i < m_ledBuffer.getLength() * portionLED; i++) {
                m_ledBuffer.setRGB(i, r, g, b);
            }
        } else {
            for (var i = 0; i < m_ledBuffer.getLength() * portionLED; i++) {}
        }

        m_led.setData(m_ledBuffer);
    }

    // sets rest of bar to a secondary color
    public void setLED_RGB_PERCENTAGE_CLR_REST(
            double portionLED, int r, int g, int b, int rb, int gb, int bb) {
        if (portionLED > 1) return;
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            if (i < m_ledBuffer.getLength() * portionLED) {
                m_ledBuffer.setRGB(i, r, g, b);
            } else {
                m_ledBuffer.setRGB(i, rb, gb, bb);
            }
        }

        m_led.setData(m_ledBuffer);
    }

    // i have no idea what to call this but to put it in a few words it colors pair numbers with one
    // color and colors the odd numbers with another color
    // thanks to terrence for the led per zone implementation
    public void setLED_RGB_Cross_etching(
            int r, int g, int b, int rb, int gb, int bb, int ledsperzone) {
        int y = 0;
        for (var i = 0; i < m_ledBuffer.getLength(); i += ledsperzone) {
            if (y % 2 == 0) {
                for (var z = 0; z < ledsperzone; z++) {
                    m_ledBuffer.setRGB(i + z, r, g, b);
                }
            } else {
                for (var z = 0; z < ledsperzone; z++) {
                    m_ledBuffer.setRGB(i + z, rb, gb, bb);
                }
            }
            y++;
        }
        y = 0;
        m_led.setData(m_ledBuffer);
    }

    public void KILL_LED_ALL() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
            m_led.setData(m_ledBuffer);
        }
    }

    public void ivans_patterns(patterens_eneum PATEnum) {
        switch (PATEnum) {
            case awtybots:
                boolean tmr = false;
                boolean f = true;
                double currentTime;

                while (f) {
                    tmr = !tmr;
                    currentTime = Timer.getFPGATimestamp(); // get current time
                    while (Timer.getFPGATimestamp() - currentTime < 0.5) {
                        // wait until 0.5 seconds have elapsed
                    }
                }

                if (tmr == true) {
                    // green
                    setLED_RGB_PERCENTAGE(1, 0, 255, 0);

                } else {
                    // gold
                    setLED_RGB_PERCENTAGE(1, 255, 191, 0);
                }
                break;
            case percentagedefault:
                setLED_RGB_PERCENTAGE(0.5, 65, 105, 225);
                break;
            case awtybotsenhanced:
                boolean tmrb = false;
                boolean fb = true;
                double currentTimeb;
                while (fb) {
                    tmrb = !tmrb;
                    currentTimeb = Timer.getFPGATimestamp(); // get current time
                    while (Timer.getFPGATimestamp() - currentTimeb < 0.5) {
                        // wait until 0.5 seconds have elapsed
                    }
                }
                if (tmrb == true) {
                    // green
                    setLED_RGB_Cross_etching(255, 191, 0, 0, 255, 0, 5);

                } else {
                    // gold
                    setLED_RGB_Cross_etching(0, 255, 0, 255, 191, 0, 5);
                }
                break;
            case awtybotsenhancedenhaced:
                boolean tmrc = false;
                boolean fc = true;
                double currentTimec;
                while (fc) {
                    tmrc = !tmrc;
                    currentTimec = Timer.getFPGATimestamp(); // get current time
                    while (Timer.getFPGATimestamp() - currentTimec < 0.5) {
                        // wait until 0.5 seconds have elapsed
                    }
                }

                if (tmrc == true) {
                    // green
                    setLED_RGB_PERCENTAGE_Strip_spicific(1, 0, 255, 0, true, 255, 191, 0);

                } else {
                    // gold
                    setLED_RGB_PERCENTAGE_Strip_spicific(1, 255, 191, 0, true, 0, 255, 0);
                }
                break;
            case cube:
                setLED_RGB_PERCENTAGE(1, 255, 0, 255);
                break;
            case cone:
                setLED_RGB_PERCENTAGE(1, 255, 191, 0);
                break;
        }
    }
}
