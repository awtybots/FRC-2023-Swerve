package frc.robot.commands.Positions.Nodes.HighNodePosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Nodes.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmElevatorMech;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class FirstPosition extends CommandBase {

    private final ArmElevatorMech s_armElevator;
    private final ElevatorMech s_elevator;

    public FirstPosition(ArmElevatorMech s_armElevatorSubsystem, ElevatorMech s_elevatorSubsystem) {
        addRequirements(s_armElevatorSubsystem, s_elevatorSubsystem);
        this.s_armElevator = s_armElevatorSubsystem;
        this.s_elevator = s_elevatorSubsystem;
    }

    @Override
    public void execute() {
        boolean isCone = RobotContainer.getIsCone();
        if (isCone) {
            s_armElevator.setExtent(Cone.HighNode.TransitionArmRotation);
            s_elevator.setHeightInches(Cone.HighNode.ElevatorPosition / 2);
        } else {
            s_armElevator.setExtent(Cube.HighNode.TransitionArmRotation);
            s_elevator.setHeightInches(Cube.HighNode.ElevatorPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return s_armElevator.atTargetExtent() && s_elevator.atTargetHeight();
    }
}
