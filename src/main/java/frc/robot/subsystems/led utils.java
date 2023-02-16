//posibly the worst LED subsystem known to man, i don't garrentee anything works here :[
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.sql.Time;
import java.util.Timer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedSubsystem extends SubsystemBase {
    
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    int length;
    
    boolean primary = true;
    public LedSubsystem(int length) {
        this.length = length;
        m_led = new AddressableLED(9);
        m_ledBuffer = new AddressableLEDBuffer(length);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setLED_RGB_PERCENTAGE(double portionLED, int r, int g, int b){
        if(portionLED > 1) return;
        for (var i = 0; i < m_ledBuffer.getLength()*portionLED; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }
    //sets rest of bar to a secondary color
    public void setLED_RGB_PERCENTAGE_CLR_REST(double portionLED, int r, int g , int b, int rb , int gb ,int bb){
        if(portionLED > 1) return;
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            if(i < m_ledBuffer*portionLED) {
                m_ledBuffer.setRGB(i, r, g, b);
            } else {
                m_ledBuffer.setRGB(i, rb, gb, bb);
            }
        }
        m_led.setData(m_ledBuffer);
    }

    public void setLED_RGB_ALL(int r, int g, int b)
    {
        double portionLED = 1;
        if(portionLED > 1) return;
        for (var i = 0; i < m_ledBuffer.getLength()*portionLED; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    public void KILL_LED_ALL(){
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
            m_led.setData(m_ledBuffer);
        }
    }
    public void ivans_patterns(patterens_eneum PATEnum) {
        switch (PATEnum) {
            case awtybots:
                System.out.print("setting to groovy awtybots patttern");
                
                boolean tmr = timer();
                if(tmr = true){
                    //green
                    setLED_RGB_ALL(34, 139, 34);
                    
                }
                else{
                    //gold
                    setLED_RGB_ALL(207, 181, 59);
                }
                break;
                case percentagedefault:
                System.out.print("setting to groovy percentage color");
                setLED_RGB_PERCENTAGE(0.5, 65, 105, 225);
                break;
            default:
                System.out.print("setting to regular color");
                setLED_RGB_ALL(65, 105, 225);
                break;
        }
    }
    private long lastTrueTime;
    private boolean timer() {
        long now= System.currentTimeMillis();
        if(primary = true){ 
            lastTrueTime=now;
            return true;
        }

        if (lastTrueTime+3000<now)
            return false;

        return true;
}
}
public enum patterens_eneum{
    awtybots,
    percentagedefault,
    
}
