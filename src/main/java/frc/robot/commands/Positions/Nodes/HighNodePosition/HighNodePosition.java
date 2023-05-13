package frc.robot.commands.Positions.Nodes.HighNodePosition;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class HighNodePosition extends SequentialCommandGroup {

    public HighNodePosition(ElevatorMech s_Elevator, ArmMech s_Arm, ClawSubsystem s_Claw) {
        if (!s_Claw.isMotorConnected()) return;
        addRequirements(s_Elevator, s_Arm, s_Claw);
        addCommands(
                new InstantCommand(() -> RobotContainer.setCurrentState(RobotContainer.State.HighNode)),
                new FirstPosition(s_Arm, s_Elevator),
                new SecondPosition(s_Elevator, s_Arm, s_Claw));
    }
}
