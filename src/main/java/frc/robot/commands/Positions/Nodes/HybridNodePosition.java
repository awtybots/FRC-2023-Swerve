package frc.robot.commands.Positions.Nodes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class HybridNodePosition extends CommandBase {

    private final ElevatorSubsystem s_elevator;
    private final ArmSubsystem s_arm;
    private final ClawSubsystem s_claw;

    public HybridNodePosition(
            ElevatorSubsystem s_elevatorSubsystem,
            ArmSubsystem s_arArmSubsystem,
            ClawSubsystem s_ClawSubsystem) {
        addRequirements(s_elevatorSubsystem, s_arArmSubsystem, s_ClawSubsystem);
        this.s_elevator = s_elevatorSubsystem;
        this.s_arm = s_arArmSubsystem;
        this.s_claw = s_ClawSubsystem;
    }

    @Override
    public void execute() {
        s_elevator.setHeight(Constants.Position.Nodes.HybridNodePosition.ElevatorPosition);
        s_arm.setRotation(Constants.Position.Nodes.HybridNodePosition.ArmPosition);
        s_claw.setRotation(Constants.Position.Nodes.HybridNodePosition.ClawPosition);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
