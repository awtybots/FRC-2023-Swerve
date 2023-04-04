package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.State;
import frc.util.LedCustomAnimations;

public class LedSubsystem extends SubsystemBase {

    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;
    private final int length;

    private static double ledCount = 0;
    private static final double LED_SPEED = 1;
    private final int stripLength;

    private boolean stop = false;

    public int[] rgb = new int[3];

    private int[] GREEN_CODE = {0, 255, 0};
    private int[] PURPLE_CODE = {255, 0, 255};
    private int[] YELLOW_CODE = {255, 255, 0};

    private LedCustomAnimations BootUp;
    private LedCustomAnimations Transitions;
    private LedCustomAnimations ConeToCube;
    private LedCustomAnimations CubeToCone;
    private LedCustomAnimations VIVELAFRANCE;
    private LedCustomAnimations Greg;
    private LedCustomAnimations IntakeCone;
    private LedCustomAnimations IntakeCube;
    private LedCustomAnimations PlaceCone;
    private LedCustomAnimations PlaceCube;
    private LedCustomAnimations ShootPiece;

    private LedCustomAnimations[] animations;

    public LedSubsystem(int LEDPort, int length) {
        this.length = length;
        this.stripLength = (int) (length / 2);

        try {
            m_led = new AddressableLED(LEDPort);
            m_ledBuffer = new AddressableLEDBuffer(length);
            m_led.setLength(m_ledBuffer.getLength());
            m_led.setData(m_ledBuffer);
            m_led.start();
        } catch (Exception e) {
            e.printStackTrace();
            stop = true;
        }

        // Creating all the animation objects
        BootUp = new LedCustomAnimations(m_led, m_ledBuffer, "BootUp2", 200, false);

        Transitions = new LedCustomAnimations(m_led, m_ledBuffer, "Transitions", 0, true);

        ConeToCube = new LedCustomAnimations(m_led, m_ledBuffer, "ConeToCube", 0, false);
        ConeToCube.end();
        
        CubeToCone = new LedCustomAnimations(m_led, m_ledBuffer, "CubeToCone", 0, false);
        CubeToCone.end();

        VIVELAFRANCE = new LedCustomAnimations(m_led, m_ledBuffer, "VIVELAFRANCE", 0, false);
        VIVELAFRANCE.end();

        Greg = new LedCustomAnimations(m_led, m_ledBuffer, "GregAmazingAnimation", 0, true);
        IntakeCone = new LedCustomAnimations(m_led, m_ledBuffer, "IntakeCone", 0, true);
        IntakeCube = new LedCustomAnimations(m_led, m_ledBuffer, "IntakeCube", 0, true);

        PlaceCone = new LedCustomAnimations(m_led, m_ledBuffer, "PlaceCone", 0, false);
        PlaceCube = new LedCustomAnimations(m_led, m_ledBuffer, "PlaceCube", 0, false);

        ShootPiece = new LedCustomAnimations(m_led, m_ledBuffer, "ShootPiece", 0, true);

        animations =
                new LedCustomAnimations[] {
                    BootUp, Transitions, ConeToCube, CubeToCone, VIVELAFRANCE, Greg, IntakeCone, IntakeCube, PlaceCone, PlaceCube
                };
    }

    // public void fillRange(int first, int last, int[] color) {
    //     for (int i = first; i < m_ledBuffer.getLength() && i < last; i++) {
    //         m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
    //     }
    // }

    // public void setColor(int c) {
    //     if (c == 0) {
    //         rgb = PURPLE_CODE;
    //     }
    //     if (c == 1) {
    //         rgb = YELLOW_CODE;
    //     }
    // }

    public void setLed(int i, int[] color) {
        if (i < 120 && i >= 0) {
            m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
        }
    }

    // public void resetAnimations(){
    //     for (LedCustomAnimations anim : animations) {
    //         anim.end();
    //         anim.setLoop(false);
    //     }
    // }

    /** Method to turn on a certain percentage of the lights of a LED strip to a certain color. */
    // public void turnOn(double portionLED, int r, int g, int b, int r2, int g2, int b2) {
    //     assert portionLED <= 1;
    //     for (var i = 0; i <= m_ledBuffer.getLength(); i++) {
    //         if (i <= m_ledBuffer.getLength() * portionLED) {
    //             m_ledBuffer.setRGB(i, r, g, b);
    //         } else {
    //             m_ledBuffer.setRGB(i, r2, g2, b2);
    //         }
    //     }
    //     m_led.setData(m_ledBuffer);
    // }

    /**
     * Method to change the amount of lights a LED strip has as green based on the area of the screen
     * that a target takes up in the Limelight Apriltag detection system.
     */
    // public void visionTrackingLED(double area) {
    //     for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    //         if (area > 17) {
    //             m_ledBuffer.setRGB(i, 0, 200, 0);
    //         } else if (area * 10 > i) {
    //             m_ledBuffer.setRGB(i, 0, 0, 200);
    //         } else {
    //             m_ledBuffer.setRGB(i, 100, 100, 100);
    //         }
    //     }
    //     m_led.setData(m_ledBuffer);
    // }

    // public void turnOff() {
    //     for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    //         m_ledBuffer.setRGB(i, 0, 0, 0);
    //     }
    //     m_led.setData(m_ledBuffer);
    // }

    // private void Animations() {
    //     for (int i = 0; i < m_ledBuffer.getLength(); i++) {
    //         setLed(i, GREEN_CODE);
    //     }

    //     if (RobotContainer.getIsCone()) setColor(1);
    //     else setColor(0);
    //     ledCount += LED_SPEED;
    //     for (int i = 0; i < stripLength; i++) {
    //         setLed(((int) (i + ledCount) % length), rgb);
    //     }

    //     if (ledCount > length) {
    //         ledCount = 0;
    //     }
    // }

    // private void GayRainbow(int offset) {
    //     final int hueShiftRate = 20;
    //     int hue = offset % 360;
    //     for (int i = 0; i < m_ledBuffer.getLength(); i++) {
    //         m_ledBuffer.setHSV(i, hue, 80, 80);
    //         hue = (hue + hueShiftRate) % 360;
    //     }
    // }

    // private int offset = 0;

    // private void RotatingRainbow() {
    //     final int speed = 2; // TODO tune led speed
    //     offset = (offset + speed) % m_ledBuffer.getLength();
    //     GayRainbow(offset);
    // }

    // boolean rainbowMode = true;

    public void setHoldAnimation(String animationName, boolean value) {
        LedCustomAnimations animation = null;
        for (LedCustomAnimations tmp_anim : animations) {
            if (tmp_anim.getName() == animationName) {
                animation = tmp_anim;
            }
        }
        if (animation == null) return;
        animation.setIsActive(value);
        if (!value) {
            animation.reset();
        }
    }

    public void setAnimation(String animationName, boolean value) {
        LedCustomAnimations animation = null;
        for (LedCustomAnimations tmp_anim : animations) {
            if (tmp_anim.getName() == animationName) {
                animation = tmp_anim;
            }
        }
        if (animation == null) return;

        if (value) {
            animation.reset();
            animation.setLoop(true);
        } else {
            animation.setLoop(false);
            animation.end();
        }
        animation.setIsActive(value);
    }

    private void SolidColor() {
        if(RobotContainer.getCurrentState() == RobotContainer.State.Shooting){
            ShootPiece.setAnimation();
        } else {
            ShootPiece.reset();
            if (RobotContainer.getIsCone()) {
                ConeToCube.reset();
                CubeToCone.setAnimation();
                if (CubeToCone.isFinished()) {
                    if (IntakeCone.isActive()) {
                        System.out.println("INTAKE CONE IS ACTIVE");
                        IntakeCone.setAnimation();
                    } else {
                        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                            setLed(i, YELLOW_CODE);
                        }
                    }
                }
            } else {
                CubeToCone.reset();
                ConeToCube.setAnimation();
                PlaceCube.reset();
                if (ConeToCube.isFinished()) {
                    if (IntakeCube.isActive()) {
                        System.out.println("INTAKE CUBE IS ACTIVE");
                        IntakeCube.setAnimation();
                    } else {
                        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                            setLed(i, PURPLE_CODE);
                        }
                    }
                }
            }
        }
    }

    @Override
    public void periodic() {
        if (stop) return;
        // TODO Use New Custom Animation Software
        if (VIVELAFRANCE.isActive()) {
            VIVELAFRANCE.setAnimation();
        } else {
            if(RobotContainer.getIsCone() && (RobotContainer.getCurrentState() == State.HighNode || RobotContainer.getCurrentState() == State.MidNode)){
                PlaceCube.reset();
                PlaceCone.setAnimation();
                if(PlaceCone.isFinished()) {
                    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                        setLed(i, YELLOW_CODE);
                    }
                }
            } else if(!RobotContainer.getIsCone() && (RobotContainer.getCurrentState() == State.HighNode || RobotContainer.getCurrentState() == State.MidNode)){
                PlaceCone.reset();
                PlaceCube.setAnimation();
                if(PlaceCube.isFinished()) {
                    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                        setLed(i, PURPLE_CODE);
                    }
                }
            } else {
                PlaceCone.reset();
                PlaceCube.reset();
                if (DriverStation.isTeleopEnabled()) {
                        SolidColor();
                } else {
                    BootUp.setAnimation();
                    if (BootUp.isFinished()) {
                        if(DriverStation.isAutonomousEnabled() && RobotContainer.getCurrentState() != State.Stow) {
                            SolidColor();
                        } else {
                            Transitions.setAnimation();
                        }
                    }
                }
            }
        }
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
}
