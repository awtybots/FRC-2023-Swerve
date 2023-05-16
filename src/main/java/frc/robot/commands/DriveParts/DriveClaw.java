package frc.robot.commands.DriveParts;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.util.Controller;

public class DriveClaw extends CommandBase {
    private final ClawSubsystem s_Claw;
    private final Controller controller;

    public DriveClaw(Controller controller, ClawSubsystem s_Claw) {
        addRequirements(s_Claw);
        this.s_Claw = s_Claw;
        this.controller = controller;
    }

    @Override
    public void execute() {
        double forward = controller.rightBumper.getAsBoolean() ? 1 : 0;
        double reverse = controller.leftBumper.getAsBoolean() ? 1 : 0;
        double rate = forward - reverse;
        s_Claw.driveClaw(rate);
    }

    @Override
    public void end(boolean interrupted) {
        s_Claw.stopClaw();
    }
}
