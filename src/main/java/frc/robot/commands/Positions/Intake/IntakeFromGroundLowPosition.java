package frc.robot.commands.Positions.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Intake.IntakeFromGroundLow;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class IntakeFromGroundLowPosition extends CommandBase {

    private final ElevatorSubsystem s_elevator;
    private final ArmSubsystem s_arm;
    private final ClawSubsystem s_claw;

    public IntakeFromGroundLowPosition(
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
        s_elevator.setHeight(IntakeFromGroundLow.ElevatorPosition);
        s_arm.setRotation(IntakeFromGroundLow.ArmPosition);
        s_claw.setRotation(IntakeFromGroundLow.ClawPosition);
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight()
                && Math.abs(s_arm.mRightArmEncoder.getPosition() - s_arm.armHeight) < Presets.ArmThreshold
                && s_claw.atTargetAngle();
    }
}
