package frc.robot.commands.Positions.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Intake.IntakeFromGroundLow;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class IntakeGroundLowPos extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmMech s_armElevator;
    private final ClawSubsystem s_claw;

    public IntakeGroundLowPos(
            ElevatorMech s_elevatorSubsystem, ArmMech s_ArmSubsystem, ClawSubsystem s_ClawSubsystem) {
        addRequirements(s_elevatorSubsystem, s_ArmSubsystem, s_ClawSubsystem);
        this.s_elevator = s_elevatorSubsystem;
        this.s_armElevator = s_ArmSubsystem;
        this.s_claw = s_ClawSubsystem;
    }

    @Override
    public void execute() {
        s_elevator.setHeight(IntakeFromGroundLow.ElevatorPosition);
        s_armElevator.setExtent(IntakeFromGroundLow.ArmPosition);
        if (!s_armElevator.atTargetExtent()) return;
        s_claw.setDegrees(IntakeFromGroundLow.ClawPosition);
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight() && s_armElevator.atTargetExtent() && s_claw.atTargetAngle();
    }
}
