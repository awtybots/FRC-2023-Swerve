package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Stow;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class StowPos extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmMech s_armElevator;
    private final ClawSubsystem s_claw;

    public StowPos(
            ElevatorMech s_elevatorSubsystem, ArmMech s_ArmSubsystem, ClawSubsystem s_ClawSubsystem) {
        addRequirements(s_elevatorSubsystem, s_ArmSubsystem, s_ClawSubsystem);
        this.s_elevator = s_elevatorSubsystem;
        this.s_armElevator = s_ArmSubsystem;
        this.s_claw = s_ClawSubsystem;
    }

    @Override
    public void execute() {
        RobotContainer.setCurrentState(RobotContainer.State.Stow);
        s_claw.setDegrees(Stow.ClawPosition);
        s_elevator.setHeight(Stow.ElevatorPosition);
        if (!s_claw.atTargetAngle()) return;
        s_armElevator.setExtent(Stow.ArmPosition);
    }

    @Override
    public void end(boolean interrupted) {
        s_armElevator.setExtent(Stow.ArmPosition);
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight() && s_armElevator.atTargetExtent() && s_claw.atTargetAngle();
    }
}
