package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;
import frc.robot.subsystems.Swerve.Swerve;
import java.util.HashMap;

public class PathPlannerAuto extends SequentialCommandGroup {
    String trajectoryJSON = "Test1";
    PathPlannerTrajectory trajectory = new PathPlannerTrajectory();
    PIDController thetaController;

    /**
     * Sequential command group that (for now only) run the {@code Test1} path using the PathPlanner
     * library. TODO: PathPlanner Auto | Make it so you can pass the HashMap of events and the
     * trajectoryJSON name.
     *
     * @param Swerve Swerve subsystem.
     * @param ElevatorSubsystem Elevator subsystem.
     * @param ArmSubsystem Arm subsystem.
     * @param ClawSubsystem Claw subsystem.
     */
    public PathPlannerAuto(
            Swerve s_Swerve,
            ElevatorSubsystem s_elevatorSubsystem,
            ArmSubsystem s_arArmSubsystem,
            ClawSubsystem s_ClawSubsystem) {
        PathPlannerTrajectory trajectory =
                PathPlanner.loadPath(trajectoryJSON, new PathConstraints(6, 4));

        thetaController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("event", new StowPosition(s_elevatorSubsystem, s_arArmSubsystem, s_ClawSubsystem));
        eventMap.put("stopEvent", new Balance(s_Swerve));

        PPSwerveControllerCommand swerveControllerCommand =
                new PPSwerveControllerCommand(
                        trajectory,
                        s_Swerve::getPose,
                        Constants.DriveConstants.kDriveKinematics,
                        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                        thetaController,
                        s_Swerve::setModuleStates,
                        s_Swerve);

        FollowPathWithEvents command =
                new FollowPathWithEvents(swerveControllerCommand, trajectory.getMarkers(), eventMap);

        addCommands(
                new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())), command);
    }
}
