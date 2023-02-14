package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
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
        // double forward = controller.dPadUp.getAsBoolean() ? 1 : 0;
        // double reverse = controller.dPadDown.getAsBoolean() ? 1 : 0;
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
