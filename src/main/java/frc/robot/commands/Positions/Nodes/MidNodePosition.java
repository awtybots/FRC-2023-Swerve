package frc.robot.commands.Positions.Nodes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class MidNodePosition extends CommandBase {

    private final ElevatorSubsystem s_elevator;
    private final ArmSubsystem s_arm;
    private final ClawSubsystem s_claw;

    public MidNodePosition(
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
        boolean isCone = RobotContainer.getIsCone();
        if (isCone) {
            s_elevator.setHeight(Constants.Position.Nodes.Cone.MidNodePosition.ElevatorPosition);
            s_arm.setRotation(Constants.Position.Nodes.Cone.MidNodePosition.ArmPosition);
            s_claw.setRotation(Constants.Position.Nodes.Cone.MidNodePosition.ClawPosition);
        } else {
            s_elevator.setHeight(Constants.Position.Nodes.Cube.MidNodePosition.ElevatorPosition);
            s_arm.setRotation(Constants.Position.Nodes.Cube.MidNodePosition.ArmPosition);
            s_claw.setRotation(Constants.Position.Nodes.Cube.MidNodePosition.ClawPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(
                                s_elevator.motors[1].getSelectedSensorPosition() - s_elevator.elevatorTargetHeight)
                        < Constants.Position.ElevatorThreshold
                && Math.abs(s_arm.mRightArmEncoder.getPosition() - s_arm.armHeight)
                        < Constants.Position.ArmThreshold
                && Math.abs(s_claw.mPivotEncoder.getPosition() - s_claw.wristHeight)
                        < Constants.Position.ClawThreshold;
    }
}
