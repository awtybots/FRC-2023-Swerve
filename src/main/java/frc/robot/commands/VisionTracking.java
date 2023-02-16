package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class VisionTracking extends CommandBase {

    private final Swerve s_Swerve;
    LimelightSubsystem s_Limelight;
    private final Boolean fieldRelative;

    private Translation2d translation;
    private double rotation;

    public VisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight){
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;
        fieldRelative = Constants.CustomConstants.fieldRelative;
    }

    @Override
    public void execute() {
        RobotContainer.isAutoTargetOn = true;
        translation = new Translation2d(0, 0);
        if (Math.abs(s_Limelight.getHorizontalOffset()) < 1) {
            rotation = 0;
            s_Swerve.drive(translation, rotation, fieldRelative);
            return;
        }
        rotation = -s_Limelight.getHorizontalOffset()/15;
        s_Swerve.drive(translation, rotation, fieldRelative);
        rotation = 0;
    }

    // @Override
    // public void end(boolean interrupted) {
    //     RobotContainer.isAutoTargetOn = false;
    // }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
    
}