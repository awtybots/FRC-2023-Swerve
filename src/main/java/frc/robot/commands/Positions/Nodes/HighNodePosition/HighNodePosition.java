package frc.robot.commands.Positions.Nodes.HighNodePosition;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class HighNodePosition extends SequentialCommandGroup {

    public HighNodePosition(
            ElevatorSubsystem s_Elevator, ArmElevatorSubsystem s_ArmElevator, ClawSubsystem s_Claw) {
        if (!s_Claw.isMotorConnected()) return;
        addRequirements(s_Elevator, s_ArmElevator, s_Claw);
        addCommands(
                new InstantCommand(() -> RobotContainer.setCurrentState(RobotContainer.State.HighNode)),
                new FirstPosition(s_ArmElevator, s_Elevator),
                new SecondPosition(s_Elevator, s_ArmElevator, s_Claw));
    }
}
