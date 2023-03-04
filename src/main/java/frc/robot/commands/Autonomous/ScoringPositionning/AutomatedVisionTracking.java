package frc.robot.commands.Autonomous.ScoringPositionning;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class AutomatedVisionTracking extends CommandBase {

    private final Swerve s_Swerve;
    private final LimelightSubsystem s_Limelight;

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
    private double ReflectiveOffset;

    public AutomatedVisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight) {
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;
        this.beta = Math.toRadians(s_Limelight.getHorizontalOffset());
        this.alpha = Math.toRadians(s_Limelight.getHorizontalRotation());

        this.rotation = 0;
        this.translation = new Translation2d(0, 0);

        this.offset = 7.5;
        this.ReflectiveOffset = 80;
    }

    public AutomatedVisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight, boolean isCone) {
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;
        this.beta = Math.toRadians(s_Limelight.getHorizontalOffset());
        this.alpha = Math.toRadians(s_Limelight.getHorizontalRotation());

        this.rotation = 0;
        this.translation = new Translation2d(0, 0);

        this.offset = 7.5;
        this.ReflectiveOffset = 80;
        s_Limelight.setPipeline(isCone ? 1 : 0);
    }
    

    private int getSign(double num) {
        if (num >= 0) return 1;
        else return -1;
    }

    @Override
    public void execute() {
        long pipelineId = s_Limelight.getPipeline();

        // April Tage
        if (pipelineId == 0) {
            if (!s_Limelight.hasTarget()) return;
            double rotation = 0;
            beta = Math.toRadians(s_Limelight.getHorizontalOffset());
            alpha = Math.toRadians(s_Limelight.getHorizontalRotation());
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
            rotation = 0;
            translation = new Translation2d(0, 0);
            if (!s_Limelight.hasTarget() && s_Limelight.getPipeline() == 1) {
                s_Limelight.setPipeline(2);
            } else if (s_Limelight.hasTarget() && s_Limelight.getPipeline() == 1) {
                alpha = Math.toRadians(s_Limelight.getSkew());
                if (Math.abs(Math.toDegrees(alpha) - ReflectiveOffset) > driveThreshold) {
                    translation = new Translation2d(0, getSign(alpha - Math.toRadians(45)) * driveSpeed);
                }
            }
            beta = Math.toRadians(s_Limelight.getHorizontalOffset());
            if (Math.abs(Math.toDegrees(beta) - offset) > rotateThreshold) {
                rotation = -getSign(beta) * rotateSpeed;
            }
            s_Swerve.drive(translation, rotation, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (s_Limelight.getPipeline() == 2) {
            s_Limelight.setPipeline(1);
        }
    }

    @Override
    public boolean isFinished() {
        if(s_Limelight.getPipeline() == 0) {
            return !s_Limelight.hasTarget()
                    || (Math.abs(Math.toDegrees(beta) - offset) < rotateThreshold
                            && Math.abs(Math.toDegrees(alpha)) < driveThreshold);
        } else if (s_Limelight.getPipeline() == 1) {
            return (Math.abs(Math.toDegrees(beta) - offset) < rotateThreshold
                            && Math.abs(Math.toDegrees(alpha) - ReflectiveOffset) < driveThreshold);
        } else {
            return !s_Limelight.hasTarget() || (Math.abs(Math.toDegrees(beta) - offset) < rotateThreshold);
        }
    }
}
