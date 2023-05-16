package frc.robot.commands.DriveParts;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.util.Controller;

public class DriveArm extends CommandBase {
    private final ArmMech s_arm;
    private final Controller controller;

    public DriveArm(Controller controller, ArmMech s_arm) {
        addRequirements(s_arm);
        this.s_arm = s_arm;
        this.controller = controller;
    }

    @Override
    public void execute() {
        double rate = -controller.getRightStickY();
        s_arm.drive(rate);
    }

    @Override
    public void end(boolean interrupted) {
        s_arm.stop();
    }
}
