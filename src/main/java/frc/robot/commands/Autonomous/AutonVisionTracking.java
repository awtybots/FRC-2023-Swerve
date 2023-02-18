package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class AutonVisionTracking extends SequentialCommandGroup {
    public AutonVisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight) {
        double beta = s_Limelight.getHorizontalOffset();
        double alpha = s_Limelight.getHorizontalRotation();
        double initial_distance = s_Limelight.getDistance();

        // TrajectoryConfig config =
        //         new TrajectoryConfig(
        //                         Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //                         Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 .setKinematics(Constants.DriveConstants.kDriveKinematics);

        // An example trajectory to follow.  All units in meters.

        double firstYTranslation = initial_distance * Math.sin(Math.toRadians(beta))
        + Math.abs(initial_distance * Math.cos(Math.toRadians(beta)) *
        Math.tan(Math.toRadians(alpha)));

        PathPlannerTrajectory trajectory1 = PathPlanner.generatePath(
                new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared),
                new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                new PathPoint(new Translation2d(0, firstYTranslation),
        Rotation2d.fromDegrees(-alpha))
        );

        // PathPlannerTrajectory exampleTrajectory =
        //         PathPlanner.generatePath(
        //                 new PathConstraints(
        //                         Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //                         Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared),
        //                 new PathPoint(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
        //                 new PathPoint(new Translation2d(0, -1), Rotation2d.fromDegrees(0)));

        // Trajectory exampleTrajectory =
        //         TrajectoryGenerator.generateTrajectory(
        //                 // Start at the origin facing the +X direction
        //                 new Pose2d(0, 0, new Rotation2d(0)),
        //                 // List.of(new Translation2d(1, 0)),
        //                 List.of(new Translation2d(0, firstYTranslation/2)),
        //                 // End 1 meters straight ahead of where we started, facing forward
        //                 new Pose2d(0, firstYTranslation, new Rotation2d(Math.toRadians(90))),
        //                 config);

        PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        Constants.AutoConstants.kPThetaController,
                        0,
                        0,
                        Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
                new SwerveControllerCommand(
                        trajectory1,
                        s_Swerve::getPose,
                        Constants.DriveConstants.kDriveKinematics,
                        xController,
                        yController,
                        thetaController,
                        s_Swerve::setModuleStates,
                        s_Swerve);

        addCommands(
                new InstantCommand(() -> s_Swerve.resetOdometry(trajectory1.getInitialPose())),
                swerveControllerCommand);
    }
}
