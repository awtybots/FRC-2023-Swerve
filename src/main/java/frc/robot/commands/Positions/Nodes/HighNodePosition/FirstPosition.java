package frc.robot.commands.Positions.Nodes.HighNodePosition;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class FirstPosition extends CommandBase {

    private final ArmSubsystem s_arm;
    private final ElevatorSubsystem s_elevator;
    private final LimelightSubsystem Limelight;

    public FirstPosition(
            ArmSubsystem s_armSubsystem,
            ElevatorSubsystem s_elevatorSubsystem,
            LimelightSubsystem s_Limelight) {
        addRequirements(s_armSubsystem, s_elevatorSubsystem);
        this.s_arm = s_armSubsystem;
        this.s_elevator = s_elevatorSubsystem;
        this.Limelight = s_Limelight;
    }

    @Override
    public void execute() {
        boolean isCone = Limelight.getPipeline() != 0;
        if (isCone) {
            s_arm.setRotation(Constants.Position.Nodes.Cone.HighNodePosition.TransitionArmRotation);
            s_elevator.setHeight(Constants.Position.Nodes.Cone.HighNodePosition.ElevatorPosition);
        } else {
            s_arm.setRotation(Constants.Position.Nodes.Cube.HighNodePosition.TransitionArmRotation);
            s_elevator.setHeight(Constants.Position.Nodes.Cube.HighNodePosition.ElevatorPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_arm.mRightArmEncoder.getPosition() - s_arm.armHeight)
                        < Constants.Position.ArmThreshold
                && Math.abs(
                                s_elevator.motors[1].getSelectedSensorPosition() - s_elevator.elevatorTargetHeight)
                        < Constants.Position.ElevatorThreshold * 10;
    }
}
