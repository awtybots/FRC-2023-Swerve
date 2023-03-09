package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class VisionTracking extends CommandBase {

    private final Swerve s_Swerve;
    private final LimelightSubsystem s_Limelight;

    private double Beta;

    // ! add to constants
    private double rotateThreshold = 2;
    private double rotateSpeed = 0.5;
    private double rotation;
    private Translation2d translation;
    private double offset;

    boolean fieldRelative = true;

    public VisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight) {
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;

        this.Beta = Math.toRadians(s_Limelight.getHorizontalOffset());

        this.offset = 7.5;
    }

    private int getSign(double num) {
        if (num >= 0) return 1;
        else return -1;
    }

    @Override
    public void execute() {
        if (!s_Limelight.hasTarget()) return;
        rotation = 0;
        translation = new Translation2d(0, 0);
        Beta = Math.toRadians(s_Limelight.getHorizontalOffset());
        if (Math.abs(Math.toDegrees(Beta) - offset) > rotateThreshold) {
            rotation = -getSign(Beta) * rotateSpeed;
        }
        translation = new Translation2d(0, 0);
        s_Swerve.drive(translation, rotation, fieldRelative);
    }

    @Override
    public boolean isFinished() {
            return !s_Limelight.hasTarget()
                    || Math.abs(Math.toDegrees(Beta) - offset) < rotateThreshold; 
        }
    }