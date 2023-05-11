package frc.robot.commands.Positions.Nodes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Nodes.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class MidNodePosition extends CommandBase {

    private final ElevatorSubsystem s_elevator;
    private final ArmElevatorSubsystem s_armElevator;
    private final ClawSubsystem s_claw;

    public MidNodePosition(
            ElevatorSubsystem s_elevatorSubsystem,
            ArmElevatorSubsystem s_ArmElevatorSubsystem,
            ClawSubsystem s_ClawSubsystem) {
        addRequirements(s_elevatorSubsystem, s_ArmElevatorSubsystem, s_ClawSubsystem);
        this.s_elevator = s_elevatorSubsystem;
        this.s_armElevator = s_ArmElevatorSubsystem;
        this.s_claw = s_ClawSubsystem;
    }

    @Override
    public void execute() {
        RobotContainer.setCurrentState(RobotContainer.State.MidNode);
        boolean isCone = RobotContainer.getIsCone();
        if (isCone) {
            s_elevator.setHeightInches(Cone.MidNode.ElevatorPosition);
            s_armElevator.setExtent(Cone.MidNode.ArmPosition);
            if (!s_armElevator.atTargetExtent()) return;
            s_claw.setDegrees(Cone.MidNode.ClawPosition);
        } else {
            s_elevator.setHeightInches(Presets.Nodes.Cube.MidNode.ElevatorPosition);
            s_armElevator.setExtent(Presets.Nodes.Cube.MidNode.ArmPosition);
            if (!s_armElevator.atTargetExtent()) return;
            s_claw.setDegrees(Presets.Nodes.Cube.MidNode.ClawPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight() && s_armElevator.atTargetExtent() && s_claw.atTargetAngle();
    }
}
