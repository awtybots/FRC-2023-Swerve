package frc.robot.commands.Positions.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets.Intake.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;

public class IntakeGroundPos extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmMech s_arm;
    private final ClawSubsystem s_claw;

    public IntakeGroundPos(ElevatorMech s_elevator, ArmMech s_arm, ClawSubsystem s_claw) {
        addRequirements(s_elevator, s_arm, s_claw);
        this.s_elevator = s_elevator;
        this.s_arm = s_arm;
        this.s_claw = s_claw;
    }

    @Override
    public void execute() {
        RobotContainer.setCurrentState(RobotContainer.State.IntakeFromGround);
        boolean isCone = RobotContainer.coneModeEnabled();
        if (isCone) {
            s_elevator.setHeight(Cone.Ground.ElevatorPosition);
            s_arm.setExtent(Cone.Ground.ArmPosition);
            if (s_arm.atTargetExtent()) {
                s_claw.setDegrees(Cone.Ground.ClawPosition);
            }
        } else {
            s_elevator.setHeight(Cube.Ground.ElevatorPosition);
            s_arm.setExtent(Cube.Ground.ArmPosition);
            if (s_arm.atTargetExtent()) {
                s_claw.setDegrees(Cube.Ground.ClawPosition);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight() && s_arm.atTargetExtent() && s_claw.atTargetAngle();
    }
}
