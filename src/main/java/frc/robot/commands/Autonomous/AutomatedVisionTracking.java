package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class AutomatedVisionTracking extends CommandBase {

    private final Swerve s_Swerve;
    LimelightSubsystem s_Limelight;
    private final Boolean fieldRelative;

    private double beta;
    private double alpha;

    private Translation2d translation1;
    private Translation2d translation2;
    private double rotation;

    private double threshold = 10;
    private double rotateSpeed = 3;
    private double driveSpeed = 4;

    public AutomatedVisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight) {
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;
        this.beta = Math.toRadians(s_Limelight.getHorizontalOffset());
        this.alpha = Math.toRadians(s_Limelight.getHorizontalRotation());
        fieldRelative = false;
    }

    private int getSign(double num){
        if(num>=0) return 1;
        else return -1;
    }

    @Override
    public void execute() {
        beta = Math.toRadians(s_Limelight.getHorizontalOffset());
        alpha = Math.toRadians(s_Limelight.getHorizontalRotation());
        if(s_Limelight.getArea() < 0.1) return;
        if(Math.abs(Math.toDegrees(beta)) > threshold) {
            s_Swerve.drive(new Translation2d(0, 0), -getSign(beta)*rotateSpeed, fieldRelative);
        } else if (Math.abs(Math.toDegrees(alpha)) > threshold){
            s_Swerve.drive(new Translation2d(0, -getSign(alpha)*driveSpeed), 0, fieldRelative);

        }
    }

    // @Override
    public boolean isFinished() {
        return Math.abs(Math.toDegrees(beta)) < threshold && Math.abs(Math.toDegrees(alpha)) < threshold;
    }

}
