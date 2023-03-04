package frc.robot.commands.Autonomous.ScoringPositionning;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class DistancePosition extends CommandBase {

    private final Swerve s_Swerve;
    private final LimelightSubsystem s_Limelight;

    public DistancePosition(Swerve s_Swerve, LimelightSubsystem s_Limelight) {
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;
    }

    @Override
    public void execute() {
        s_Swerve.drive(new Translation2d(1, 0), 0, false);
    }

    // @Override
    public boolean isFinished() {
        return !s_Limelight.hasTarget() || s_Swerve.getOutputCurrent() > 10;
    }
}
