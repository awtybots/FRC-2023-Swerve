package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutomatedVisionTracking extends CommandBase {

    private final Swerve s_Swerve;
    LimelightSubsystem s_Limelight;
    private final Boolean fieldRelative;

    private final double beta;
    private final double alpha;
    private final double initial_distance;

    private Translation2d translation1;
    private Translation2d translation2;
    private double rotation;

    public AutomatedVisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight){
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;
        this.beta = s_Limelight.getHorizontalOffset();
        this.alpha = s_Limelight.getHorizontalRotation();
        this.initial_distance = s_Limelight.getDistance();
        fieldRelative = Constants.CustomConstants.fieldRelative;
    }

    @Override
    public void execute() {
        s_Swerve.resetOdometry(new Pose2d());
        translation1 = new Translation2d(initial_distance*Math.sin(beta*Math.PI/180) + Math.abs(initial_distance*Math.cos(beta*Math.PI/180)) * Math.tan(alpha*Math.PI/180), 0);
        rotation = -1*alpha*Math.PI/180;
        s_Swerve.drive(translation1, rotation, fieldRelative);
        s_Swerve.resetOdometry(new Pose2d());
        translation2 = new Translation2d(s_Limelight.getDistance() - Constants.LimeLightConstants.distanceToTarget, 0);
        s_Swerve.drive(translation2, 0, fieldRelative);
        rotation = -1*s_Limelight.getHorizontalOffset()/2;
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
    
}
