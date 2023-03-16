package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Intake.IntakeFromGroundLow;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class NewPosition extends CommandBase {

    private final ElevatorSubsystem s_elevator;
    private final ArmSubsystem s_arm;
    private final ClawSubsystem s_claw;

    private final double ElevatorHeight;
    private final double ArmAngle;
    private final double ClawAngle;

    public NewPosition(
            ElevatorSubsystem s_elevatorSubsystem,
            ArmSubsystem s_arArmSubsystem,
            ClawSubsystem s_ClawSubsystem,
            
            double ElevatorHeight,
            double ArmAngle,
            double ClawAngle
            ) {
        addRequirements(s_elevatorSubsystem, s_arArmSubsystem, s_ClawSubsystem);
        this.s_elevator = s_elevatorSubsystem;
        this.s_arm = s_arArmSubsystem;
        this.s_claw = s_ClawSubsystem;

        this.ElevatorHeight = ElevatorHeight;
        this.ArmAngle = ArmAngle;
        this.ClawAngle = ClawAngle;
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
