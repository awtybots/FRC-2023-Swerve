package frc.robot.commands.Positions.Nodes.HighNodePosition;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;

// public class HighNodePosition extends CommandBase {

//     private final ElevatorSubsystem s_elevator;
//     private final ArmSubsystem s_arm;
//     private final ClawSubsystem s_claw;

//     private Command firstCommand;
//     private Command secondCommand;

//     public HighNodePosition(
//             ElevatorSubsystem s_elevatorSubsystem,
//             ArmSubsystem s_arArmSubsystem,
//             ClawSubsystem s_ClawSubsystem) {
//         addRequirements(s_elevatorSubsystem, s_arArmSubsystem, s_ClawSubsystem);
//         this.s_elevator = s_elevatorSubsystem;
//         this.s_arm = s_arArmSubsystem;
//         this.s_claw = s_ClawSubsystem;
//     }

//     @Override
//     public void execute() {

//     }

//     @Override
//     public boolean isFinished() {
//         return true;
//         // return secondCommand.isFinished();
//         // return Math.abs(
//         //                         s_elevator.motors[1].getSelectedSensorPosition() -
// s_elevator.elevatorTargetHeight)
//         //                 < Constants.Position.ElevatorThreshold
//         //         && Math.abs(s_arm.mRightArmEncoder.getPosition() - s_arm.armHeight)
//         //                 < Constants.Position.ArmThreshold
//         //         && Math.abs(s_claw.mPivotEncoder.getPosition() - s_claw.wristHeight)
//         //                 < Constants.Position.ClawThreshold;
//     }
// }

public class HighNodePosition extends SequentialCommandGroup {

    public HighNodePosition(ElevatorSubsystem s_Elevator, ArmSubsystem s_Arm, ClawSubsystem s_Claw) {
        addRequirements(s_Elevator, s_Arm, s_Claw);
        addCommands(
                new FirstPosition(s_Arm, s_Elevator), new SecondPosition(s_Elevator, s_Arm, s_Claw));
    }
}
