package frc.robot.commands.Positions.Nodes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Nodes.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class MidNodePos extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmMech s_arm;
    private final ClawSubsystem s_claw;

    public MidNodePos(ElevatorMech s_elevator, ArmMech s_arm, ClawSubsystem s_claw) {
        addRequirements(s_elevator, s_arm, s_claw);
        this.s_elevator = s_elevator;
        this.s_arm = s_arm;
        this.s_claw = s_claw;
    }

    @Override
    public void execute() {
        RobotContainer.setCurrentState(RobotContainer.State.MidNode);

        if (RobotContainer.coneModeEnabled()) {
            s_elevator.setHeight(Cone.MidNode.ElevatorSetpoint);
            s_arm.setExtent(Cone.MidNode.ArmSetpoint);

            if (s_arm.atTargetExtent()) {
                s_claw.setDegrees(Cone.MidNode.ClawSetpoint);
            }
        } else {
            s_elevator.setHeight(Cube.MidNode.ElevatorSetpoint);
            s_arm.setExtent(Cube.MidNode.ArmSetpoint);

            if (s_arm.atTargetExtent()) {
                s_claw.setDegrees(Cube.MidNode.ClawSetpoint);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight() && s_arm.atTargetExtent() && s_claw.atTargetAngle();
    }
}
