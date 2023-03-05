package frc.robot.commands.Autonomous.ScoringPositionning;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class PlaceSetup extends SequentialCommandGroup {

    public PlaceSetup(Swerve s_Swerve, LimelightSubsystem s_Limelight) {
        addCommands(
                new AutomatedVisionTracking(s_Swerve, s_Limelight),
                new DistancePosition(s_Swerve, s_Limelight));
    }

    public PlaceSetup(Swerve s_Swerve, LimelightSubsystem s_Limelight, boolean isCone) {
        addCommands(
                new AutomatedVisionTracking(s_Swerve, s_Limelight, isCone),
                new DistancePosition(s_Swerve, s_Limelight));
    }
}
