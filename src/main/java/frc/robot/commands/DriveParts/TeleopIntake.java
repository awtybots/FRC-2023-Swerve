package frc.robot.commands.DriveParts;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.IntakeMech;
import frc.util.Controller;

public class TeleopIntake extends CommandBase {
    private final IntakeMech s_Intake;
    private final Controller controller;

    public TeleopIntake(Controller controller, IntakeMech s_Intake) {
        addRequirements(s_Intake);
        this.s_Intake = s_Intake;
        this.controller = controller;
    }

    @Override
    public void execute() {
        double forward = 0;
        double reverse = 0;

        double Right = controller.getRightTrigger();
        double Left = controller.getLeftTrigger();

        if (RobotContainer.getIsCone()) {
            forward = Right > 0.1 ? Right : 0;
            reverse = Left > 0.2 ? Left : 0;
        } else {
            forward = Left > 0.1 ? Left : 0;
            reverse = Right > 0.1 ? Right : 0;
        }
        double rate = forward - reverse;
        s_Intake.intake(rate, false);
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
