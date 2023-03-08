package frc.robot.commands.Positions.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

public class IntakeFromGroundPosition extends CommandBase {

    private final ElevatorSubsystem s_elevator;
    private final ArmSubsystem s_arm;
    private final ClawSubsystem s_claw;
    private final LimelightSubsystem Limelight;

    public IntakeFromGroundPosition(
            ElevatorSubsystem s_elevatorSubsystem,
            ArmSubsystem s_arArmSubsystem,
            ClawSubsystem s_ClawSubsystem,
            LimelightSubsystem s_Limelight) {
        addRequirements(s_elevatorSubsystem, s_arArmSubsystem, s_ClawSubsystem);
        this.s_elevator = s_elevatorSubsystem;
        this.s_arm = s_arArmSubsystem;
        this.s_claw = s_ClawSubsystem;
        this.Limelight = s_Limelight;
    }

    @Override
    public void execute() {
        boolean isCone = Limelight.getPipeline() != 0;
        if(isCone){
            s_elevator.setHeight(Constants.Position.Intake.Cone.IntakeFromGroundPosition.ElevatorPosition);
            s_arm.setRotation(Constants.Position.Intake.Cone.IntakeFromGroundPosition.ArmPosition);
            s_claw.setRotation(Constants.Position.Intake.Cone.IntakeFromGroundPosition.ClawPosition);
        } else {
            s_elevator.setHeight(Constants.Position.Intake.Cube.IntakeFromGroundPosition.ElevatorPosition);
            s_arm.setRotation(Constants.Position.Intake.Cube.IntakeFromGroundPosition.ArmPosition);
            s_claw.setRotation(Constants.Position.Intake.Cube.IntakeFromGroundPosition.ClawPosition);
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(
                                s_elevator.motors[1].getSelectedSensorPosition() - s_elevator.elevatorTargetHeight)
                        < Constants.Position.ElevatorThreshold
                && Math.abs(s_arm.mRightArmEncoder.getPosition() - s_arm.armHeight)
                        < Constants.Position.ArmThreshold
                && Math.abs(s_claw.mPivotEncoder.getPosition() - s_claw.wristHeight)
                        < Constants.Position.ClawThreshold;
    }
}
