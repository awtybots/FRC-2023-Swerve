package frc.robot.commands.DriveParts;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MechanicalParts.PistonSubsystem;
import frc.util.Controller;

public class ToggleIntakeMode extends CommandBase {
    private final PistonSubsystem sPiston;
    private final Controller controller;

    public ToggleIntakeMode(Controller controller, PistonSubsystem sPiston) {
        addRequirements(sPiston);
        this.sPiston = sPiston;
        this.controller = controller;
    }

    @Override
    public void execute() {
        if (controller.dPadLeft.getAsBoolean()) {
            sPiston.Close();
        } else if (controller.dPadRight.getAsBoolean()) {
            sPiston.open();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
