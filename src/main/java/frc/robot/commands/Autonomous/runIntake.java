package frc.robot.commands.Autonomous;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.IntakeSubsystem;

public class runIntake extends CommandBase{
    
    private final IntakeSubsystem s_intake;
    private final LimelightSubsystem Limelight;

    public runIntake(

            IntakeSubsystem IntakeSubsystem,
            LimelightSubsystem s_LimelightSubsystem) {
        addRequirements(IntakeSubsystem);
        this.s_intake = IntakeSubsystem;
        this.Limelight = s_LimelightSubsystem;

    }

    @Override
    public void initialize() {
    }
    @Override
    public void execute() {
        boolean isCone = Limelight.getPipeline() != 0;
        s_intake.intake(isCone ? 1.5 : -1.5, true);
        }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end (boolean interrupted) {
    }



}
