package frc.robot.commands.Autonomous.ShootPiece;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Nodes.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmElevatorMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class Position extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmElevatorMech s_armElevator;
    private final ClawSubsystem s_claw;

    public Position(
            ArmElevatorMech s_armElevatorSubsystem,
            ElevatorMech s_elevatorSubsystem,
            ClawSubsystem s_clawSubsystem) {
        addRequirements(s_armElevatorSubsystem, s_elevatorSubsystem);
        this.s_armElevator = s_armElevatorSubsystem;
        this.s_elevator = s_elevatorSubsystem;
        this.s_claw = s_clawSubsystem;
    }

    @Override
    public void execute() {
        RobotContainer.setCurrentState(RobotContainer.State.Shooting);
        s_armElevator.setExtent(Cube.ShootCube.ArmPosition);
        s_elevator.setHeight(Cube.ShootCube.ElevatorPosition);
        s_claw.setDegrees(Cube.ShootCube.ClawPosition);
    }

    @Override
    public boolean isFinished() {
        return s_armElevator.atTargetExtent() && s_elevator.atTargetHeight() && s_claw.atTargetAngle();
    }
}
