package frc.robot.commands;

import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionTracking extends CommandBase {

    private final Swerve s_Swerve;
    private final LimelightSubsystem s_Limelight;
    private final boolean fieldRelative;
    private final boolean openLoop;

    private Translation2d translation;
    private double rotation;

    public VisionTracking(LimelightSubsystem s_Limelight, Swerve s_Swerve, Boolean fieldRelative, Boolean openLoop){
        this.s_Limelight = s_Limelight;
        this.s_Swerve = s_Swerve;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        translation = new Translation2d(0, 0);
        rotation = .5;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
    
}
