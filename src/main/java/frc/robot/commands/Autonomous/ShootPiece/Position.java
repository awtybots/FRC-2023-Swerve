package frc.robot.commands.Autonomous.ShootPiece;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Nodes.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class Position extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmMech s_arm;
    private final ClawSubsystem s_claw;

    public Position(ArmMech s_arm, ElevatorMech s_elevator, ClawSubsystem s_claw) {
        addRequirements(s_arm, s_elevator);
        this.s_arm = s_arm;
        this.s_elevator = s_elevator;
        this.s_claw = s_claw;
    }

    @Override
    public void initialize() {
        RobotContainer.setCurrentState(RobotContainer.State.Shooting);
        s_arm.setExtent(Cube.ShootCube.ArmSetpoint);
        s_elevator.setHeight(Cube.ShootCube.ElevatorSetpoint);
        s_claw.setDegrees(Cube.ShootCube.ClawSetpoint);
    }

    @Override
    public boolean isFinished() {
        return s_arm.atTargetExtent() && s_elevator.atTargetHeight() && s_claw.atTargetAngle();
    }
}
