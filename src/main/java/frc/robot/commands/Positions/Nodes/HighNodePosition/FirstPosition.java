package frc.robot.commands.Positions.Nodes.HighNodePosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Nodes.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class FirstPosition extends CommandBase {

    private final ArmMech s_arm;
    private final ElevatorMech s_elevator;

    public FirstPosition(ArmMech s_arm, ElevatorMech s_elevator) {
        addRequirements(s_arm, s_elevator);
        this.s_arm = s_arm;
        this.s_elevator = s_elevator;
    }

    @Override
    public void execute() {
        boolean isCone = RobotContainer.coneModeEnabled();
        if (isCone) {
            s_arm.setExtent(Cone.TransitionArmRotation);
            s_elevator.setHeight(Cone.HighNode.ElevatorSetpoint / 2);
        } else {
            s_arm.setExtent(Cube.TransitionArmRotation);
            s_elevator.setHeight(Cube.HighNode.ElevatorSetpoint);
        }
    }

    @Override
    public boolean isFinished() {
        return s_arm.atTargetExtent() && s_elevator.atTargetHeight();
    }
}
