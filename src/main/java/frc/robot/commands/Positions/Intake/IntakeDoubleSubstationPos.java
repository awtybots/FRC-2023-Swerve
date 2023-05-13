package frc.robot.commands.Positions.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Intake.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class IntakeDoubleSubstationPos extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmMech s_armElevator;
    private final ClawSubsystem s_claw;

    public IntakeDoubleSubstationPos(
            ElevatorMech s_elevatorSubsystem,
            ArmMech s_armElevatorSubsystem,
            ClawSubsystem s_ClawSubsystem) {
        addRequirements(s_elevatorSubsystem, s_armElevatorSubsystem, s_ClawSubsystem);
        this.s_elevator = s_elevatorSubsystem;
        this.s_armElevator = s_armElevatorSubsystem;
        this.s_claw = s_ClawSubsystem;
    }

    @Override
    public void execute() {
        boolean isCone = RobotContainer.coneModeEnabled();
        if (isCone) {
            s_elevator.setHeight(Cone.IntakeFromHumanPlayer.ElevatorPosition);
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
