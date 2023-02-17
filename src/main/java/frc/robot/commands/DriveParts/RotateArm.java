package frc.robot.commands.DriveParts;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.util.Controller;

public class RotateArm extends CommandBase {
    private final ArmSubsystem s_arm;
    private final Controller controller;

    public RotateArm(Controller controller, ArmSubsystem s_armSubsystem) {
        addRequirements(s_armSubsystem);
        this.s_arm = s_armSubsystem;
        this.controller = controller;
    }

    @Override
    public void execute() {
        // double forward = controller.dPadUp.getAsBoolean() ? 1 : 0;
        //  double reverse = controller.dPadDown.getAsBoolean() ? 1 : 0;
        // double rate = forward - reverse;
        double rate = -controller.getRightStickY();

        s_arm.drive(rate);
    }

    @Override
    public void end(boolean interrupted) {
        s_arm.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
