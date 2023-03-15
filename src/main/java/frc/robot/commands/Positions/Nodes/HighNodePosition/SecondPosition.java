package frc.robot.commands.Positions.Nodes.HighNodePosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Nodes.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class SecondPosition extends CommandBase {

    private final ElevatorSubsystem s_elevator;
    private final ArmSubsystem s_arm;
    private final ClawSubsystem s_claw;

    public SecondPosition(ElevatorSubsystem s_Elevator, ArmSubsystem s_Arm, ClawSubsystem s_Claw) {
        addRequirements(s_Elevator, s_Arm, s_Claw);
        this.s_elevator = s_Elevator;
        this.s_arm = s_Arm;
        this.s_claw = s_Claw;
    }

    @Override
    public void execute() {
        boolean isCone = RobotContainer.getIsCone();
        if (isCone) {
            s_elevator.setHeightInches(Cone.HighNode.ElevatorPosition);
            s_arm.setDegrees(Cone.HighNode.ArmPosition);
            s_claw.setDegrees(Cone.HighNode.ClawPosition);
        } else {
            s_elevator.setHeight(Cube.HighNode.ElevatorPosition);
            s_arm.setRotation(Cube.HighNode.ArmPosition);
            s_claw.setRotation(Cube.HighNode.ClawPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight()
                && Math.abs(s_arm.mRightArmEncoder.getPosition() - s_arm.armHeight) < Presets.ArmThreshold;
        // && Math.abs(s_claw.mPivotEncoder.getPosition() - s_claw.wristHeight)
        //         < Constants.Position.ClawThreshold;
    }
}
