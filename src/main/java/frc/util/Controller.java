package frc.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.*;

public class Controller {
    private final XboxController controller;
    private final double kDeadzoneStick = 0.1;
    private final double kDeadzoneTrigger = 0.0;

    public final JoystickButton buttonA, buttonX, buttonY, buttonB;
    public final JoystickButton buttonBack, buttonStart;
    public final JoystickButton leftBumper, rightBumper;
    public final JoystickButton leftStickClick, rightStickClick;

    // D-Pad
    public POVButton dPadUp, dPadRight, dPadDown, dPadLeft;

    /** Allows using the triggers as buttons, for example <pre> mController.leftTrigger.whenPressed(new CommandToRun());*/
    public Trigger leftTrigger, rightTrigger;

    /** @param port The port index on the Driver Station that the controller is plugged into. */
    public Controller(int port) {
        controller = new XboxController(port);

        buttonA = createButton(XboxController.Button.kA.value);
        buttonX = createButton(XboxController.Button.kX.value);
        buttonY = createButton(XboxController.Button.kY.value);
        buttonB = createButton(XboxController.Button.kB.value);
        buttonBack = createButton(XboxController.Button.kBack.value);
        buttonStart = createButton(XboxController.Button.kStart.value);

        leftBumper = createButton(XboxController.Button.kLeftBumper.value);
        rightBumper = createButton(XboxController.Button.kRightBumper.value);

        leftStickClick = createButton(XboxController.Button.kLeftStick.value);
        rightStickClick = createButton(XboxController.Button.kRightStick.value);

        dPadUp = new POVButton(controller, 0);
        dPadRight = new POVButton(controller, 90);
        dPadDown = new POVButton(controller, 180);
        dPadLeft = new POVButton(controller, 270);

        leftTrigger = new Trigger(() -> getLeftTrigger() > kDeadzoneTrigger);
        rightTrigger = new Trigger(() -> getRightTrigger() > kDeadzoneTrigger);
    }

    /** The X (left/right) position of the right joystick on the controller from -1.0 to 1.0 */
    public double getRightStickX() {
        return deadzone(controller.getRightX(), kDeadzoneStick);
    }

    /** The Y (up/down) position of the right joystick on the controller from -1.0 to 1.0 */
    public double getRightStickY() {
        return deadzone(-controller.getRightY(), kDeadzoneStick);
    }

    /** The X (left/right) position of the left joystick on the controller from -1.0 to 1.0 */
    public double getLeftStickX() {
        return deadzone(controller.getLeftX(), kDeadzoneStick);
    }

    /** The Y (up/down) position of the left joystick on the controller from -1.0 to 1.0 */
    public double getLeftStickY() {
        return deadzone(-controller.getLeftY(), kDeadzoneStick);
    }

    /** How much the left trigger on the controller is pressed from 0.0 to 1.0 */
    public double getLeftTrigger() {
        return deadzone(controller.getLeftTriggerAxis(), kDeadzoneTrigger);
    }

    /** How much the right trigger on the controller is pressed from 0.0 to 1.0 */
    public double getRightTrigger() {
        return deadzone(controller.getRightTriggerAxis(), kDeadzoneTrigger);
    }

    // --- Utilities --- //
    private JoystickButton createButton(int buttonID) {
        return new JoystickButton(this.controller, buttonID);
    }

    private double deadzone(double x, double dz) {
        if (Math.abs(x) > dz) return (x - dz * Math.signum(x)) / (1.0 - dz);
        else return 0.0;
    }
}