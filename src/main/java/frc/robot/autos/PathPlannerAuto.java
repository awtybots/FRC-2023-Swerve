package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.Swerve;
import java.util.HashMap;

public class PathPlannerAuto extends SequentialCommandGroup {
    private final PIDController thetaController;

    /**
     * Sequential command group that runs a given PathPlanner path with an event Hashmap triggering
     * commands.
     *
     * @param PathPlannerTrajectory The loaded PathPlanner trajectory.
     * @param Swerve Swerve subsystem.
     * @param HashMap<String,Command> Hashmap of events along the loaded trajectory.
     */
    public PathPlannerAuto(
            PathPlannerTrajectory trajectory, Swerve s_Swerve, HashMap<String, Command> eventMap) {

        thetaController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveControllerCommand =
                new PPSwerveControllerCommand(
                        trajectory,
                        s_Swerve::getPose,
                        Constants.DriveConstants.kDriveKinematics,
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                        thetaController,
                        s_Swerve::setModuleStates,
                        false,
                        s_Swerve);

        FollowPathWithEvents command =
                new FollowPathWithEvents(swerveControllerCommand, trajectory.getMarkers(), eventMap);

        addCommands(
                new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialHolonomicPose())), command);
                // new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(trajectory.getInitialState().poseMeters.getTranslation(), trajectory.getInitialState().holonomicRotation))), command);
    }

    //     public PathPlannerAuto(PathPlannerTrajectory trajectory, Swerve s_Swerve) {

    //         thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //         final PPSwerveControllerCommand swerveControllerCommand =
    //                 new PPSwerveControllerCommand(
    //                         trajectory,
    //                         s_Swerve::getPose,
    //                         Constants.DriveConstants.kDriveKinematics,
    //                         new PIDController(Constants.AutoConstants.kPXController, 0, 0),
    //                         new PIDController(Constants.AutoConstants.kPYController, 0, 0),
    //                         thetaController,
    //                         s_Swerve::setModuleStates,
    //                         s_Swerve);

    //         addCommands(
    //                 new InstantCommand(() ->
    // s_Swerve.resetOdometry(trajectory.getInitialHolonomicPose())),
    //                 swerveControllerCommand);
    //     }
}
