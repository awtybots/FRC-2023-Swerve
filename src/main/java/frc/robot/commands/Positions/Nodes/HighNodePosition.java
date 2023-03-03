package frc.robot.commands.Positions.Nodes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class HighNodePosition extends CommandBase {

    private final ElevatorSubsystem s_elevator;
    private final ArmSubsystem s_arm;
    private final ClawSubsystem s_claw;

    public HighNodePosition(
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
        if (s_arm.getCanCoder().getRadians() > (Math.PI / 2) && s_elevator.getDistance() >= 0.7) {
            s_elevator.setHeight(0.6);
            try {
                Thread.sleep(500);
            }
            catch (InterruptedException e1) {
                e1.printStackTrace();
            }
            s_arm.setRotation(Constants.Position.Nodes.TransitionHighNodePosition.ArmPosition);
            s_claw.setRotation(Constants.Position.Nodes.TransitionHighNodePosition.ClawPosition);
            try {
                Thread.sleep(500);
            }
            catch (InterruptedException e1) {
                e1.printStackTrace();
            }
        } else {
            s_arm.setRotation(Constants.Position.Nodes.TransitionHighNodePosition.ArmPosition);
            s_claw.setRotation(Constants.Position.Nodes.TransitionHighNodePosition.ClawPosition);
            try {
                Thread.sleep(500);
            }
            catch (InterruptedException e1) {
                e1.printStackTrace();
            }
        }

        s_elevator.setHeight(Constants.Position.Nodes.HighNodePosition.ElevatorPosition);
        s_arm.setRotation(Constants.Position.Nodes.HighNodePosition.ArmPosition);
        s_claw.setRotation(Constants.Position.Nodes.HighNodePosition.ClawPosition);
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
