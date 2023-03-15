package frc.robot.commands.Positions.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Intake.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class IntakeFromGroundPosition extends CommandBase {

    private final ElevatorSubsystem s_elevator;
    private final ArmSubsystem s_arm;
    private final ClawSubsystem s_claw;

    public IntakeFromGroundPosition(
            ElevatorSubsystem s_elevatorSubsystem,
            ArmSubsystem s_arArmSubsystem,
            ClawSubsystem s_ClawSubsystem) {
        addRequirements(s_elevatorSubsystem, s_arArmSubsystem, s_ClawSubsystem);
        this.s_elevator = s_elevatorSubsystem;
        this.s_arm = s_arArmSubsystem;
        this.s_claw = s_ClawSubsystem;
    }

    @Override
    public void execute() {
        boolean isCone = RobotContainer.getIsCone();
        if (isCone) {
            s_elevator.setHeight(Cone.IntakeFromGround.ElevatorPosition);
            s_arm.setRotation(Cone.IntakeFromGround.ArmPosition);
            s_claw.setRotation(Cone.IntakeFromGround.ClawPosition);
        } else {
            s_elevator.setHeight(Cube.IntakeFromGround.ElevatorPosition);
            s_arm.setRotation(Cube.IntakeFromGround.ArmPosition);
            s_claw.setRotation(Cube.IntakeFromGround.ClawPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight()
                && (Math.abs(s_arm.mRightArmEncoder.getPosition() - s_arm.armHeight) < Presets.ArmThreshold)
                && (Math.abs(s_claw.mPivotEncoder.getPosition() - s_claw.wristHeight)
                        < Presets.ClawThreshold);
    }
}
