package frc.robot.commands.Positions.Nodes.HighNodePosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Nodes.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class SecondPosition extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmMech s_arm;
    private final ClawSubsystem s_claw;

    public SecondPosition(ElevatorMech s_elevator, ArmMech s_arm, ClawSubsystem s_claw) {
        addRequirements(s_elevator, s_arm, s_claw);
        this.s_elevator = s_elevator;
        this.s_arm = s_arm;
        this.s_claw = s_claw;
    }

    @Override
    public void execute() {
        boolean isCone = RobotContainer.coneModeEnabled();
        if (isCone) {
            s_elevator.setHeight(Cone.HighNode.ElevatorSetpoint);
            s_arm.setExtent(Cone.HighNode.ArmSetpoint);
            s_claw.setDegrees(Cone.HighNode.ClawSetpoint);
        } else {
            s_elevator.setHeight(Cube.HighNode.ElevatorSetpoint);
            s_arm.setExtent(Cube.HighNode.ArmSetpoint);
            s_claw.setDegrees(Cube.HighNode.ClawSetpoint);
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight() && s_arm.atTargetExtent();
    }
}
