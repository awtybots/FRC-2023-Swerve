package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Positions.Nodes.HighNodePosition.HighNodePosition;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.IntakeSubsystem;
import frc.robot.subsystems.MechanicalParts.PistonSubsystem;

public class Place extends SequentialCommandGroup {

    public Place(
            ClawSubsystem s_Claw,
            ArmSubsystem s_Arm,
            ElevatorSubsystem s_Elevator,
            IntakeSubsystem s_Intake,
            PistonSubsystem s_Piston,
            boolean isCone) {
        addRequirements(s_Claw, s_Arm, s_Elevator, s_Intake);
        // ! only isCone = true done, not finished cube and possible intake?
        if (isCone) {
            addCommands(
                    new HighNodePosition(s_Elevator, s_Arm, s_Claw),
                    new InstantCommand(() -> s_Piston.open()),
                    new InstantCommand(() -> s_Intake.intake(1, IntakeConstants.IntakeEjectionTime)),
                    new InstantCommand(() -> s_Intake.stopIntake()));
        } else {
            addCommands(new InstantCommand());
        }
    }
}
