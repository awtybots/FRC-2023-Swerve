package frc.robot.commands.Autonomous.ShootPiece;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Nodes.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class Position extends CommandBase {

    private final ElevatorSubsystem s_elevator;
    private final ArmElevatorSubsystem s_armElevator;
    private final ClawSubsystem s_claw;

    public Position(
            ArmElevatorSubsystem s_armElevatorSubsystem,
            ElevatorSubsystem s_elevatorSubsystem,
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
        s_elevator.setHeightInches(Cube.ShootCube.ElevatorPosition);
        s_claw.setDegrees(Cube.ShootCube.ClawPosition);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(s_armElevator.mArmEncoder.getPosition() - s_armElevator.armExtent)
                        < Presets.ArmThreshold
                && s_elevator.atTargetHeight()
                && s_claw.atTargetAngle();
    }
}
