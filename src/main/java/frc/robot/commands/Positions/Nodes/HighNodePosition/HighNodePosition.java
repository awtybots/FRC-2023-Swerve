package frc.robot.commands.Positions.Nodes.HighNodePosition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class HighNodePosition extends SequentialCommandGroup {


    public HighNodePosition(
            ElevatorSubsystem s_Elevator,
            ArmSubsystem s_Arm,
            ClawSubsystem s_Claw) {
        addRequirements(s_Elevator, s_Arm, s_Claw);
        addCommands(
                new FirstPosition(s_Arm, s_Elevator),
                new SecondPosition(s_Elevator, s_Arm, s_Claw));
    }
}
