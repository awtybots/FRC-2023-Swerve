package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class VisionTracking extends CommandBase {

    private final Swerve s_Swerve;
    LimelightSubsystem s_Limelight;
    private final Boolean fieldRelative;

    private Translation2d translation;
    private double rotation;

    public VisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight) {
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;
        fieldRelative = Constants.CustomConstants.fieldRelative;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        translation = new Translation2d(0, 0);
        if (Math.abs(s_Limelight.getHorizontalOffset()) < 1) {
            rotation = 0;
            s_Swerve.drive(translation, rotation, fieldRelative);
            return;
        }
        rotation = -s_Limelight.getHorizontalOffset() / 15;
        s_Swerve.drive(translation, rotation, fieldRelative);
        rotation = 0;
    }

    // ? Why was this commented out? It just makes it impossible to regain Teleop control if commented
    // out.
    @Override
    public void end(boolean interrupted) {}

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }

}
