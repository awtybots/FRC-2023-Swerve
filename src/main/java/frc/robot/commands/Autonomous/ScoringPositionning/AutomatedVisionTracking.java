package frc.robot.commands.Autonomous.ScoringPositionning;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;
import frc.robot.Constants;

public class AutomatedVisionTracking extends CommandBase {

    private final Swerve s_Swerve;
    private final LimelightSubsystem s_Limelight;

    private double aprilBeta;
    private double aprilAlpha;
    private double reflectiveBeta;
    private double reflectiveAlpha;

    // ! add to constants
    private double rotateThreshold = 2;
    private double driveThreshold = 3;
    private double rotateSpeed = 0.5;
    private double reflectiveThreshold = 3;
    private double driveSpeed = 0.5;
    private double rotation;
    private Translation2d translation;
    private double offset;
    private double ReflectiveOffset;
    boolean fieldRelative = false;

    public AutomatedVisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight) {
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;

        this.aprilBeta = Math.toRadians(s_Limelight.getHorizontalOffset());
        this.aprilAlpha = Math.toRadians(s_Limelight.getHorizontalRotation());
        this.reflectiveBeta = Math.toRadians(s_Limelight.getHorizontalOffset());
        this.reflectiveAlpha = Math.toRadians(s_Limelight.getHorizontalRotation());

        this.offset = 7.5;
        this.ReflectiveOffset = 84;
    }

    public AutomatedVisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight, boolean isCone) {
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;

        this.aprilBeta = Math.toRadians(s_Limelight.getHorizontalOffset());
        this.aprilAlpha = Math.toRadians(s_Limelight.getHorizontalRotation());
        this.reflectiveBeta = Math.toRadians(s_Limelight.getHorizontalOffset());
        this.reflectiveAlpha = Math.toRadians(s_Limelight.getSkew());

        this.offset = 7.5;
        this.ReflectiveOffset = 84;
        s_Limelight.setPipeline(isCone ? 1 : 0);
    }

    private int getSign(double num) {
        if (num >= 0) return 1;
        else return -1;
    }

    private void AprilTagLogic() {
        if (!s_Limelight.hasTarget()) return;
        rotation = 0;
        aprilBeta = Math.toRadians(s_Limelight.getHorizontalOffset());
        aprilAlpha = Math.toRadians(s_Limelight.getHorizontalRotation());
        // Rotate until beta is right
        if (Math.abs(Math.toDegrees(aprilBeta) - offset) > rotateThreshold) {
            rotation = -getSign(aprilBeta) * rotateSpeed;
        }
        // Strafe until Alpha is right
        if (Math.abs(Math.toDegrees(aprilAlpha)) > driveThreshold && Constants.CustomConstants.VisionTrackingStrafe) {
            translation = new Translation2d(0, getSign(aprilAlpha) * driveSpeed);
        }
        s_Swerve.drive(translation, rotation, fieldRelative);
    }

    private void ReflectiveTapeLogic() {
        rotation = 0;
        reflectiveBeta = Math.toRadians(s_Limelight.getHorizontalOffset());
        // Sets pipeline to number 2 (one reflective tape) if there is no target
        if (!s_Limelight.hasTarget() && s_Limelight.getPipeline() == 1) {
            s_Limelight.setPipeline(2);
            // Strafe until Alpha is right
        } else if (s_Limelight.hasTarget() && s_Limelight.getPipeline() == 1) {
            reflectiveBeta = Math.toRadians(s_Limelight.getSkew());
            if (Math.abs(Math.toDegrees(reflectiveBeta) - ReflectiveOffset) > reflectiveThreshold && Constants.CustomConstants.VisionTrackingStrafe) {
                translation =
                        new Translation2d(0, -getSign(reflectiveBeta - Math.toRadians(45)) * driveSpeed);
            }
        }
        // Rotate until beta is right
        if (Math.abs(Math.toDegrees(reflectiveBeta) - offset) > reflectiveThreshold) {
            rotation = -getSign(reflectiveBeta) * rotateSpeed;
        }
        s_Swerve.drive(translation, rotation, fieldRelative);
    }

    @Override
    public void execute() {
        long pipelineId = s_Limelight.getPipeline();
        translation = new Translation2d(0, 0);
        if (pipelineId == 0) {
            AprilTagLogic();
        } else {
            ReflectiveTapeLogic();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // reverts to the two reflective tapes mode if it's set to only detect one
        if (s_Limelight.getPipeline() == 2) {
            s_Limelight.setPipeline(1);
        }
    }

    @Override
    public boolean isFinished() {
        if (s_Limelight.getPipeline() == 0) {
            return !s_Limelight.hasTarget()
                    || Math.abs(Math.toDegrees(aprilBeta) - offset) < rotateThreshold
                            && Math.abs(Math.toDegrees(aprilAlpha)) < driveThreshold;
        } else if (s_Limelight.getPipeline() == 1) {
            return (Math.abs(Math.toDegrees(aprilBeta) - offset) < reflectiveThreshold
                    && Math.abs(Math.toDegrees(aprilAlpha) - ReflectiveOffset) < reflectiveThreshold);
        } else {
            return !s_Limelight.hasTarget()
                    || (Math.abs(Math.toDegrees(aprilBeta) - offset) < rotateThreshold);
        }
    }
}
