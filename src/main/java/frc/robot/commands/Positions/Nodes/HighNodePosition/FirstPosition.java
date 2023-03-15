package frc.robot.commands.Positions.Nodes.HighNodePosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Nodes.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class FirstPosition extends CommandBase {

    private final ArmSubsystem s_arm;
    private final ElevatorSubsystem s_elevator;

    public FirstPosition(ArmSubsystem s_armSubsystem, ElevatorSubsystem s_elevatorSubsystem) {
        addRequirements(s_armSubsystem, s_elevatorSubsystem);
        this.s_arm = s_armSubsystem;
        this.s_elevator = s_elevatorSubsystem;
    }

    @Override
    public void execute() {
        boolean isCone = RobotContainer.getIsCone();
        if (isCone) {
            s_arm.setDegrees(Cone.HighNode.TransitionArmRotation);
            s_elevator.setHeightInches(Cone.HighNode.ElevatorPosition);
        } else {
            s_arm.setRotation(Cube.HighNode.TransitionArmRotation);
            s_elevator.setHeight(Cube.HighNode.ElevatorPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_arm.mRightArmEncoder.getPosition() - s_arm.armHeight) < Presets.ArmThreshold
                && s_elevator.atTargetHeight();
    }
}
