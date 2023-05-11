package frc.robot.commands.Positions.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Intake.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class IntakeFromHumanPlayerPosition extends CommandBase {

    private final ElevatorSubsystem s_elevator;
    private final ArmElevatorSubsystem s_armElevator;
    private final ClawSubsystem s_claw;

    public IntakeFromHumanPlayerPosition(
            ElevatorSubsystem s_elevatorSubsystem,
            ArmElevatorSubsystem s_armElevatorSubsystem,
            ClawSubsystem s_ClawSubsystem) {
        addRequirements(s_elevatorSubsystem, s_armElevatorSubsystem, s_ClawSubsystem);
        this.s_elevator = s_elevatorSubsystem;
        this.s_armElevator = s_armElevatorSubsystem;
        this.s_claw = s_ClawSubsystem;
    }

    @Override
    public void execute() {
        boolean isCone = RobotContainer.getIsCone();
        if (isCone) {
            s_elevator.setHeightInches(Cone.IntakeFromHumanPlayer.ElevatorPosition);
            s_armElevator.setExtent(Cone.IntakeFromHumanPlayer.ArmPosition);
            if (!s_armElevator.atTargetExtent()) return;
            s_claw.setDegrees(Cone.IntakeFromHumanPlayer.ClawPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight() && s_armElevator.atTargetExtent() && s_claw.atTargetAngle();
    }
}
