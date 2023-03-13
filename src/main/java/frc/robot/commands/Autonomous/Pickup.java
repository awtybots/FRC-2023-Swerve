package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Positions.Intake.IntakeFromGroundPosition;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.IntakeSubsystem;

public class Pickup extends SequentialCommandGroup {

    public Pickup(
            ClawSubsystem s_Claw,
            ArmSubsystem s_Arm,
            ElevatorSubsystem s_Elevator,
            IntakeSubsystem s_Intake) {
        // addRequirements(s_Claw, s_Arm, s_Elevator, s_Intake);
        // ! only isCone = true done, not finished cube and possible intake?
        addCommands(
                new IntakeFromGroundPosition(s_Elevator, s_Arm, s_Claw),
                new Intake(s_Intake, RobotContainer.getIsCone()).withTimeout(1.5),
                new StowPosition(s_Elevator, s_Arm, s_Claw));
    }
}
