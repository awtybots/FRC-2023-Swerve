package frc.robot.commands.Positions.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Intake.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmElevatorMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class IntakeFromSlidingHumanPlayerPosition extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmElevatorMech s_armElevator;
    private final ClawSubsystem s_claw;

    public IntakeFromSlidingHumanPlayerPosition(
            ElevatorMech s_elevatorSubsystem,
            ArmElevatorMech s_armElevatorSubsystem,
            ClawSubsystem s_ClawSubsystem) {
        addRequirements(s_elevatorSubsystem, s_armElevatorSubsystem, s_ClawSubsystem);
        this.s_elevator = s_elevatorSubsystem;
        this.s_armElevator = s_armElevatorSubsystem;
        this.s_claw = s_ClawSubsystem;
    }

    @Override
    public void execute() {
        RobotContainer.setCurrentState(RobotContainer.State.IntakeFromSlidingHumanPlayer);
        boolean isCone = RobotContainer.getIsCone();
        if (isCone) {
            s_elevator.setHeightInches(Cone.IntakeFromSlidingHumanPlayer.ElevatorPosition);
            s_armElevator.setExtent(Cone.IntakeFromSlidingHumanPlayer.ArmPosition);
            if (!s_armElevator.atTargetExtent()) return;
            s_claw.setDegrees(Cone.IntakeFromSlidingHumanPlayer.ClawPosition);
        } else {
            s_elevator.setHeightInches(Cube.IntakeFromSlidingHumanPlayer.ElevatorPosition);
            s_armElevator.setExtent(Cube.IntakeFromSlidingHumanPlayer.ArmPosition);
            if (!s_armElevator.atTargetExtent()) return;
            s_claw.setDegrees(Cube.IntakeFromSlidingHumanPlayer.ClawPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight() && s_armElevator.atTargetExtent() && s_claw.atTargetAngle();
    }
}
