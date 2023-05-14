package frc.robot.commands.Positions;

import static frc.robot.Constants.Presets.Stow.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class StowPos extends CommandBase {

    private final ClawSubsystem s_claw;
    private final ElevatorMech s_elevator;
    private final ArmMech s_arm;

    public StowPos(ElevatorMech s_elevator, ArmMech s_arm, ClawSubsystem s_claw) {
        addRequirements(s_elevator, s_arm, s_claw);
        this.s_elevator = s_elevator;
        this.s_arm = s_arm;
        this.s_claw = s_claw;
    }

    @Override
    public void execute() {
        RobotContainer.setCurrentState(RobotContainer.State.Stow);
        s_claw.setDegrees(ClawPosition);
        s_elevator.setHeight(ElevatorPosition);

        if (s_claw.atTargetAngle()) s_arm.setExtent(ArmPosition);
    }

    @Override
    public void end(boolean interrupted) {
        s_arm.setExtent(ArmPosition);
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight() && s_arm.atTargetExtent() && s_claw.atTargetAngle();
    }
}
