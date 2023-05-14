package frc.robot.commands.Positions.Nodes.HighNodePosition;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class HighNodePosition extends SequentialCommandGroup {

    public HighNodePosition(ElevatorMech s_elevator, ArmMech s_arm, ClawSubsystem s_claw) {
        if (!s_claw.isMotorConnected()) return;
        addRequirements(s_elevator, s_arm, s_claw);
        addCommands(
                new InstantCommand(() -> RobotContainer.setCurrentState(RobotContainer.State.HighNode)),
                new FirstPosition(s_arm, s_elevator),
                new SecondPosition(s_elevator, s_arm, s_claw));
    }
}
