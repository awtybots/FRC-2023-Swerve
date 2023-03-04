package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class AutomatedVisionTracking extends CommandBase {

    private final Swerve s_Swerve;
    LimelightSubsystem s_Limelight;

    private double beta;
    private double alpha;

    // ! add to constants
    private double rotateThreshold = 3;
    private double driveThreshold = 3;
    private double rotateSpeed = 1;
    private double driveSpeed = 0.5;
    private double rotation;
    private Translation2d translation;
    private double offset;

    public AutomatedVisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight) {
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;
        this.beta = Math.toRadians(s_Limelight.getHorizontalOffset());
        this.alpha = Math.toRadians(s_Limelight.getHorizontalRotation());

        this.rotation = 0;
        this.translation = new Translation2d(0, 0);

        this.offset = 7.5;
        // this.offset = 0;
    }

    private int getSign(double num) {
        if (num >= 0) return 1;
        else return -1;
    }

    @Override
    public void execute() {
        long pipelineId = s_Limelight.getPipeline();

        // April Tage
        if(pipelineId == 0) {
            double rotation = 0;
            beta = Math.toRadians(s_Limelight.getHorizontalOffset());
            alpha = Math.toRadians(s_Limelight.getHorizontalRotation());
            if (s_Limelight.getArea() < 0.1) return;
            if (Math.abs(Math.toDegrees(beta) - offset) > rotateThreshold) {
                rotation = -getSign(beta) * rotateSpeed;
            }
            if (Math.abs(Math.toDegrees(alpha)) > driveThreshold) {
                translation = new Translation2d(0, getSign(alpha) * driveSpeed);
            }
            s_Swerve.drive(translation, rotation, false);
        } 
        // Reflective Tape
        else {

        }
    }

    // @Override
    public boolean isFinished() {
        return s_Limelight.getArea() < 0.1
                || (Math.abs(Math.toDegrees(beta) - offset) < rotateThreshold
                        && Math.abs(Math.toDegrees(alpha)) < driveThreshold);
    }
}
