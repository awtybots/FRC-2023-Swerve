package frc.robot.commands.DriveParts;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MechanicalParts.ArmElevatorSubsystem;
import frc.util.Controller;

public class DriveArmElevator extends CommandBase {
    private final ArmElevatorSubsystem s_armElevator;
    private final Controller controller;

    public DriveArmElevator(Controller controller, ArmElevatorSubsystem s_armElevatorSubsystem) {
        addRequirements(s_armElevatorSubsystem);
        this.s_armElevator = s_armElevatorSubsystem;
        this.controller = controller;
    }

    @Override
    public void execute() {
        // double forward = controller.rightTrigger.getAsBoolean() ? 1 : 0;
        // double reverse = controller.leftTrigger.getAsBoolean() ? 1 : 0;
        // double rate = forward - reverse;
        // double rate = -controller.getRightStickY();

        // s_armElevator.drive(rate);
    }

    @Override
    public void end(boolean interrupted) {
        s_armElevator.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
