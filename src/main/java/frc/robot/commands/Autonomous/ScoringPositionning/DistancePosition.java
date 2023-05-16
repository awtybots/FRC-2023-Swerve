package frc.robot.commands.Autonomous.ScoringPositionning;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class DistancePosition extends CommandBase {

    private final Swerve s_swerve;
    private final LimelightSubsystem s_limelight;

    public DistancePosition(Swerve s_Swerve, LimelightSubsystem s_Limelight) {
        addRequirements(s_Swerve);
        this.s_swerve = s_Swerve;
        this.s_limelight = s_Limelight;
    }

    @Override
    public void execute() {
        s_swerve.drive(new Translation2d(0.5, 0), 0, false);
    }

    @Override
    public boolean isFinished() {
        return !s_limelight.hasTarget() || s_swerve.getOutputCurrent() > 40;
    }

    @Override
    public void end(boolean interrupted) {
        s_swerve.drive(new Translation2d(0, 0), 0, true);
    }
}
