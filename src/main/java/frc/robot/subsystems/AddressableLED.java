package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class AddressableLED extends SubsystemBase {
    
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    @Override
    public void robotInit() {
        // PWM port 9
        // Must be a PWM header, not MXP or DIO
        m_led = new AddressableLED(9);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(120);
        m_led.setLength(m_ledBuffer.getLength());

        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            if (area > 17) {
                m_ledBuffer.setRGB(i, 0, 200, 0);
            }
            
            else if (area*10 > i){
                m_ledBuffer.setRGB(i, 0, 0, 200);
            }
            else{
                m_ledBuffer.setRGB(i, 100, 100, 100);
            }
         }
         
         m_led.setData(m_ledBuffer);
        }
    }
}