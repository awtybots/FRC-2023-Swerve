package frc.robot.commands.Positions.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Presets;
import frc.robot.Constants.Presets.Intake.*;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class IntakeFromSlidingHumanPlayerPosition extends CommandBase {

    private final ElevatorSubsystem s_elevator;
    private final ArmSubsystem s_arm;
    private final ClawSubsystem s_claw;

    public IntakeFromSlidingHumanPlayerPosition(
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
            s_elevator.setHeight(Cone.IntakeFromSlidingHumanPlayer.ElevatorPosition);
            s_arm.setRotation(Cone.IntakeFromSlidingHumanPlayer.ArmPosition);
            s_claw.setRotation(Cone.IntakeFromSlidingHumanPlayer.ClawPosition);
        } else {
            s_elevator.setHeight(Cube.IntakeFromSlidingHumanPlayer.ElevatorPosition);
            s_arm.setRotation(Cube.IntakeFromSlidingHumanPlayer.ArmPosition);
            s_claw.setRotation(Cube.IntakeFromSlidingHumanPlayer.ClawPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return s_elevator.atTargetHeight()
                && Math.abs(s_arm.mRightArmEncoder.getPosition() - s_arm.armHeight) < Presets.ArmThreshold
                && s_claw.atTargetAngle();
    }
}
