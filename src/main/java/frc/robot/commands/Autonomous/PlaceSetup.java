package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class PlaceSetup extends SequentialCommandGroup {

    public PlaceSetup(Swerve s_Swerve, LimelightSubsystem s_Limelight) {
        addCommands(new AutomatedVisionTracking(s_Swerve, s_Limelight));
    }


}