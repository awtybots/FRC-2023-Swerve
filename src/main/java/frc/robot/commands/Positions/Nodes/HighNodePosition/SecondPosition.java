package frc.robot.commands.Positions.Nodes.HighNodePosition;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Nodes.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class SecondPosition extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmMech s_armElevator;
    private final ClawSubsystem s_claw;

    public SecondPosition(
            ElevatorMech s_Elevator, ArmMech s_armElevatorSubsystem, ClawSubsystem s_Claw) {
        addRequirements(s_Elevator, s_armElevatorSubsystem, s_Claw);
        this.s_elevator = s_Elevator;
        this.s_armElevator = s_armElevatorSubsystem;
        this.s_claw = s_Claw;
    }

    @Override
    public void execute() {
        boolean isCone = RobotContainer.coneModeEnabled();
        if (isCone) {
            s_elevator.setHeight(Cone.HighNode.ElevatorPosition);
            s_armElevator.setExtent(Cone.HighNode.ArmPosition);
            s_claw.setDegrees(Cone.HighNode.ClawPosition);
        } else {
            s_elevator.setHeight(Cube.HighNode.ElevatorPosition);
            s_armElevator.setExtent(Cube.HighNode.ArmPosition);
            s_claw.setDegrees(Cube.HighNode.ClawPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight() && s_armElevator.atTargetExtent();
        // && Math.abs(s_claw.mPivotEncoder.getPosition() - s_claw.wristHeight)
        //         < Constants.Position.ClawThreshold;
    }
}
