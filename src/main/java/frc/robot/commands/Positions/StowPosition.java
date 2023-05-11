package frc.robot.commands.Positions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Stow;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class StowPosition extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmElevatorSubsystem s_armElevator;
    private final ClawSubsystem s_claw;

    public StowPosition(
            ElevatorMech s_elevatorSubsystem,
            ArmElevatorSubsystem s_ArmElevatorSubsystem,
            ClawSubsystem s_ClawSubsystem) {
        addRequirements(s_elevatorSubsystem, s_ArmElevatorSubsystem, s_ClawSubsystem);
        this.s_elevator = s_elevatorSubsystem;
        this.s_armElevator = s_ArmElevatorSubsystem;
        this.s_claw = s_ClawSubsystem;
    }

    @Override
    public void execute() {
        RobotContainer.setCurrentState(RobotContainer.State.Stow);
        s_claw.setDegrees(Stow.ClawPosition);
        s_elevator.setHeightInches(Stow.ElevatorPosition);
        if (!s_claw.atTargetAngle()) return;
        s_armElevator.setExtent(Stow.ArmPosition);
    }

    @Override
    public void end(boolean interrupted) {
        s_armElevator.setExtent(Stow.ArmPosition);
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight()
                && Math.abs(s_armElevator.mArmEncoder.getPosition() - s_armElevator.armExtent)
                        < Presets.ArmThreshold
                && s_claw.atTargetAngle();
    }
}
