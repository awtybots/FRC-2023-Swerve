package frc.robot.autos;

import java.util.HashMap;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class PathPlannerTest extends SequentialCommandGroup {
    String trajectoryJSON = "pathplanner/generatedJSON/New_New_Path.wpilib.json";
    Trajectory trajectory = new Trajectory();

    public PathPlannerTest(Swerve s_Swerve){
        String trajectoryPath = Filesystem.getDeployDirectory().toPath().toString();
        PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryPath, new PathConstraints(6, 4));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        // TODO: Figure out FollowPathWithEvents
        // FollowPathWithEvents followpath = new FollowPathWithEvents(
        //     getPathFollowingCommand(trajectory),
        //     trajectory.getMarkers(),
        //     eventMap
        // );

        var thetaController =
            new PIDController(
                Constants.AutoConstants.kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        PPSwerveControllerCommand swerveControllerCommand =
            new PPSwerveControllerCommand(
                trajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}