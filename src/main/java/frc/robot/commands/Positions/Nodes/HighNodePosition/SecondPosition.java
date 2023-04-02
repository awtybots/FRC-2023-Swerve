package frc.robot.commands.Positions.Nodes.HighNodePosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Nodes.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class SecondPosition extends CommandBase {

    private final ElevatorSubsystem s_elevator;
    private final ArmElevatorSubsystem s_armElevator;
    private final ClawSubsystem s_claw;

    public SecondPosition(
            ElevatorSubsystem s_Elevator,
            ArmElevatorSubsystem s_armElevatorSubsystem,
            ClawSubsystem s_Claw) {
        addRequirements(s_Elevator, s_armElevatorSubsystem, s_Claw);
        this.s_elevator = s_Elevator;
        this.s_armElevator = s_armElevatorSubsystem;
        this.s_claw = s_Claw;
    }

    @Override
    public void execute() {
        boolean isCone = RobotContainer.getIsCone();
        if (isCone) {
            s_elevator.setHeightInches(Cone.HighNode.ElevatorPosition);
            s_armElevator.setExtent(Cone.HighNode.ArmPosition);
            s_claw.setDegrees(Cone.HighNode.ClawPosition);
        } else {
            s_elevator.setHeightInches(Cube.HighNode.ElevatorPosition);
            s_armElevator.setExtent(Cube.HighNode.ArmPosition);
            s_claw.setDegrees(Cube.HighNode.ClawPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight()
                && Math.abs(s_armElevator.mArmEncoder.getPosition() - s_armElevator.armExtent)
                        < Presets.ArmThreshold;
        // && Math.abs(s_claw.mPivotEncoder.getPosition() - s_claw.wristHeight)
        //         < Constants.Position.ClawThreshold;
    }
}
