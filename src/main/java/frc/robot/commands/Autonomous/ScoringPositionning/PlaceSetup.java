package frc.robot.commands.Autonomous.ScoringPositionning;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class PlaceSetup extends SequentialCommandGroup {

    public PlaceSetup(Swerve s_swerve, LimelightSubsystem s_limelight) {
        addCommands(
                new AutomatedVisionTracking(s_swerve, s_limelight),
                new DistancePosition(s_swerve, s_limelight));
    }
}
