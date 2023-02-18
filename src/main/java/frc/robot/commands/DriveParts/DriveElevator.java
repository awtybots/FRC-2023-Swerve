package frc.robot.commands.DriveParts;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;
import frc.util.Controller;

public class DriveElevator extends CommandBase {
    private final ElevatorSubsystem s_elevator;
    private final Controller controller;

    public DriveElevator(Controller controller, ElevatorSubsystem s_elevatorSubsystem) {
        addRequirements(s_elevatorSubsystem);
        this.s_elevator = s_elevatorSubsystem;
        this.controller = controller;
    }

    @Override
    public void execute() {
        // double forward = controller.rightBumper.getAsBoolean() ? 1 : 0;
        // double reverse = controller.leftBumper.getAsBoolean() ? 1 : 0;
        // double rate = forward - reverse;
        double rate = controller.getLeftStickY();
        s_elevator.drive(rate);
    }

    @Override
    public void end(boolean interrupted) {
        s_elevator.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
