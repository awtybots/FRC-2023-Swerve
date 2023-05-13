package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        IntakeCone = new LedCustomAnimations(m_led, m_ledBuffer, "IntakeGreen", 0, true);
        IntakeCube = new LedCustomAnimations(m_led, m_ledBuffer, "IntakeGreen", 0, true);

        PlaceCone = new LedCustomAnimations(m_led, m_ledBuffer, "PlaceCone", 0, false);
        PlaceCube = new LedCustomAnimations(m_led, m_ledBuffer, "PlaceCube", 0, false);

        ShootPiece = new LedCustomAnimations(m_led, m_ledBuffer, "ShootPiece", 0, true);

        animations =
                new LedCustomAnimations[] {
                    BootUp,
                    Transitions,
                    ConeToCube,
                    CubeToCone,
                    VIVELAFRANCE,
                    Greg,
                    IntakeCone,
                    IntakeCube,
                    PlaceCone,
                    PlaceCube
                };
    }

    public void setLed(int i, int[] color) {
        if (i < 120 && i >= 0) {
            m_ledBuffer.setRGB(i, color[0], color[1], color[2]);
        }
    }

    public void setHoldAnimation(String animationName, boolean value) {
        LedCustomAnimations animation = findAnim(animationName);
        if (animation == null) return;

        animation.setIsActive(value);

        if (!value) animation.reset();
    }

    public Command animationRun(String name, boolean loop) {
        return new InstantCommand(
                () -> {
                    this.setAnimation(name, loop);
                });
    }

    private LedCustomAnimations findAnim(String name) {
        for (var anim : this.animations) {
            if (anim.getName() == name) return anim;
        }
        return null;
    }

    public void setAnimation(String animationName, boolean loop) {
        LedCustomAnimations animation = findAnim(animationName);
        if (animation == null) return;

        if (loop) {
            animation.reset();
            animation.setLoop(true);
        } else {
            animation.setLoop(false);
            animation.end();
        }
        animation.setIsActive(loop);
    }

    private void SolidColor() {
        if (RobotContainer.getCurrentState() == RobotContainer.State.Shooting) {
            ShootPiece.setAnimation();
        } else {
            ShootPiece.reset();
            if (RobotContainer.coneModeEnabled()) {
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
            if (RobotContainer.coneModeEnabled()
                    && (RobotContainer.getCurrentState() == State.HighNode
                            || RobotContainer.getCurrentState() == State.MidNode)) {
                PlaceCube.reset();
                PlaceCone.setAnimation();
                if (PlaceCone.isFinished()) {
                    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                        setLed(i, YELLOW_CODE);
                    }
                }
            } else if (!RobotContainer.coneModeEnabled()
                    && (RobotContainer.getCurrentState() == State.HighNode
                            || RobotContainer.getCurrentState() == State.MidNode)) {
                PlaceCone.reset();
                PlaceCube.setAnimation();
                if (PlaceCube.isFinished()) {
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
                        if (DriverStation.isAutonomousEnabled()
                                && RobotContainer.getCurrentState() != State.Stow) {
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
