package frc.robot.commands.Positions.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Intake.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmElevatorMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class IntakeFromGroundPosition extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmElevatorMech s_armElevator;
    private final ClawSubsystem s_claw;

    public IntakeFromGroundPosition(
            ElevatorMech s_elevatorSubsystem,
            ArmElevatorMech s_armElevatorSubsystem,
            ClawSubsystem s_ClawSubsystem) {
        addRequirements(s_elevatorSubsystem, s_armElevatorSubsystem, s_ClawSubsystem);
        this.s_elevator = s_elevatorSubsystem;
        this.s_armElevator = s_armElevatorSubsystem;
        this.s_claw = s_ClawSubsystem;
    }

    @Override
    public void execute() {
        RobotContainer.setCurrentState(RobotContainer.State.IntakeFromGround);
        boolean isCone = RobotContainer.coneModeEnabled();
        if (isCone) {
            s_elevator.setHeight(Cone.IntakeFromGround.ElevatorPosition);
            s_armElevator.setExtent(Cone.IntakeFromGround.ArmPosition);
            if (!s_armElevator.atTargetExtent()) return;
            s_claw.setDegrees(Cone.IntakeFromGround.ClawPosition);
        } else {
            s_elevator.setHeight(Cube.IntakeFromGround.ElevatorPosition);
            s_armElevator.setExtent(Cube.IntakeFromGround.ArmPosition);
            if (!s_armElevator.atTargetExtent()) return;
            s_claw.setDegrees(Cube.IntakeFromGround.ClawPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight() && s_armElevator.atTargetExtent() && s_claw.atTargetAngle();
    }
}
