package frc.robot.commands.Positions.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Intake.IntakeFromGroundLow;
import frc.robot.subsystems.MechanicalParts.ArmElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class IntakeFromGroundLowPosition extends CommandBase {

    private final ElevatorSubsystem s_elevator;
    private final ArmElevatorSubsystem s_armElevator;
    private final ClawSubsystem s_claw;

    public IntakeFromGroundLowPosition(
            ElevatorSubsystem s_elevatorSubsystem,
            ArmElevatorSubsystem s_ArmElevatorSubsystem,
            ClawSubsystem s_ClawSubsystem) {
        addRequirements(s_elevatorSubsystem, s_ArmElevatorSubsystem, s_ClawSubsystem);
        this.s_elevator = s_elevatorSubsystem;
        this.s_armElevator = s_ArmElevatorSubsystem;
        this.s_claw = s_ClawSubsystem;
    }

    @Override
    public void execute() {
        s_elevator.setHeightInches(IntakeFromGroundLow.ElevatorPosition);
        s_armElevator.setExtent(IntakeFromGroundLow.ArmPosition);
        if (!s_armElevator.atTargetExtent()) return;
        s_claw.setDegrees(IntakeFromGroundLow.ClawPosition);
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight()
                && Math.abs(s_armElevator.mArmEncoder.getPosition() - s_armElevator.armExtent)
                        < Presets.ArmThreshold
                && s_claw.atTargetAngle();
    }
}
