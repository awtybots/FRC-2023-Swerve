package frc.robot.commands.Positions.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Intake.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class IntakeDoubleSubstationPos extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmMech s_arm;
    private final ClawSubsystem s_claw;

    public IntakeDoubleSubstationPos(
            ElevatorMech s_elevator,
            ArmMech s_arm,
            ClawSubsystem s_claw) {
        addRequirements(s_elevator, s_arm, s_claw);
        this.s_elevator = s_elevator;
        this.s_arm = s_arm;
        this.s_claw = s_claw;
    }

    @Override
    public void execute() {
        if (RobotContainer.coneModeEnabled()) {
            s_elevator.setHeight(Cone.DoubleSubstation.ElevatorPosition);
            s_arm.setExtent(Cone.DoubleSubstation.ArmPosition);
            if (!s_arm.atTargetExtent()) return;
            s_claw.setDegrees(Cone.DoubleSubstation.ClawPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight() && s_arm.atTargetExtent() && s_claw.atTargetAngle();
    }
}
