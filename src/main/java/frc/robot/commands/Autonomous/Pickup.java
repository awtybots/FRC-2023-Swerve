package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Positions.Intake.IntakeGroundPos;
import frc.robot.commands.Positions.StowPos;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;
import frc.robot.subsystems.MechanicalParts.IntakeMech;

public class Pickup extends SequentialCommandGroup {

    public Pickup(
            boolean isCone,
            ClawSubsystem s_Claw,
            ArmMech s_Arm,
            ElevatorMech s_Elevator,
            IntakeMech s_Intake) {
        // addRequirements(s_Claw, s_Arm, s_Elevator, s_Intake);
        // ! only isCone = true done, not finished cube and possible intake?
        addCommands(
                new InstantCommand(() -> RobotContainer.enableConeMode(isCone)),
                new IntakeGroundPos(s_Elevator, s_Arm, s_Claw),
                new AutonIntakeCurrentLimit(s_Intake).withTimeout(0.2),
                new StowPos(s_Elevator, s_Arm, s_Claw));
    }
}
