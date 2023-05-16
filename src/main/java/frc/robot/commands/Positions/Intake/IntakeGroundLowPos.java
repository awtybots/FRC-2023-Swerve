package frc.robot.commands.Positions.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Intake.GroundLow;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class IntakeGroundLowPos extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmMech s_arm;
    private final ClawSubsystem s_claw;

    public IntakeGroundLowPos(ElevatorMech s_elevator, ArmMech s_arm, ClawSubsystem s_claw) {
        addRequirements(s_elevator, s_arm, s_claw);
        this.s_elevator = s_elevator;
        this.s_arm = s_arm;
        this.s_claw = s_claw;
    }

    @Override
    public void execute() {
        s_elevator.setHeight(GroundLow.ElevatorPosition);
        s_arm.setExtent(GroundLow.ArmPosition);
        if (s_arm.atTargetExtent()) {
            s_claw.setDegrees(GroundLow.ClawPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight() && s_arm.atTargetExtent() && s_claw.atTargetAngle();
    }
}
