package frc.robot.commands.Positions.Nodes.HighNodePosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
            s_elevator.setHeight(Constants.Position.Nodes.Cone.HighNodePosition.ElevatorPosition);
            s_arm.setRotation(Constants.Position.Nodes.Cone.HighNodePosition.ArmPosition);
            s_claw.setRotation(Constants.Position.Nodes.Cone.HighNodePosition.ClawPosition);
        } else {
            s_elevator.setHeight(Constants.Position.Nodes.Cube.HighNodePosition.ElevatorPosition);
            s_arm.setRotation(Constants.Position.Nodes.Cube.HighNodePosition.ArmPosition);
            s_claw.setRotation(Constants.Position.Nodes.Cube.HighNodePosition.ClawPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight()
                && Math.abs(s_arm.mRightArmEncoder.getPosition() - s_arm.armHeight)
                        < Constants.Position.ArmThreshold;
        // && Math.abs(s_claw.mPivotEncoder.getPosition() - s_claw.wristHeight)
        //         < Constants.Position.ClawThreshold;
    }
}
