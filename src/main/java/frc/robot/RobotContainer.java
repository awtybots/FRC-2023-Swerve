// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Autonomous.*;
import frc.robot.commands.Autonomous.Balance.*;
import frc.robot.commands.Autonomous.ShootPiece.*;
import frc.robot.commands.DriveParts.*;
import frc.robot.commands.Positions.Intake.*;
import frc.robot.commands.Positions.Nodes.HighNodePosition.HighNodePosition;
import frc.robot.commands.Positions.Nodes.MidNodePosition;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MechanicalParts.*;
import frc.robot.subsystems.Swerve.Swerve;
import frc.util.AutonManager;
import frc.util.Controller;
import java.util.HashMap;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private final AutonManager autonManager;

    private final Swerve Swerve = new Swerve();
    private final LedSubsystem Led = new LedSubsystem(0, 72);

    private final LimelightSubsystem Limelight = new LimelightSubsystem();

    private final ElevatorSubsystem Elevator = new ElevatorSubsystem();
    private final ArmElevatorSubsystem Arm = new ArmElevatorSubsystem();
    private final ClawSubsystem Claw = new ClawSubsystem();
    private final IntakeSubsystem Intake = new IntakeSubsystem(Led);

    private static boolean isCone = Constants.DefaultConfig.isCone;
    private static double angleOffset = 0;

    public enum State {
        Stow,

        IntakeFromGround,
        IntakeFromSlidingHumanPlayer,

        Shooting,
        HighNode,
        MidNode,

        Balance
    }

    private static State currentState = State.Stow;

    private final Controller driver = new Controller(0);
    private final Controller operator = new Controller(1);

    private final HashMap<String, Command> eventMap = new HashMap<>();

    private final String[] autonChoices =
            new String[] {
                "GyroTest",
                "LeftPlacePickup",
                "LeftPlacePickupPlace",
                "LeftPlacePickupShootBalance",
                "MiddlePlace",
                "MiddlePlaceBalance",
                "MiddlePlaceExitBalance",
                "RightPlacePickup",
                "RightPlacePickupPlace",
                "RightPlacePickupShootBalance",
            };

    public final SwerveAutoBuilder autoBuilder =
            new SwerveAutoBuilder(
                    Swerve::getPose,
                    Swerve::resetOdometry,
                    Constants.Drivetrain.kDriveKinematics,
                    new PIDConstants(Constants.Auton.kPXYController, 0.0, 0.0),
                    new PIDConstants(Constants.Auton.kPThetaController, 0.0, 0.0),
                    Swerve::setModuleStates,
                    eventMap,
                    true,
                    Swerve);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        autonManager = new AutonManager(Limelight);
        eventAssignment();
        addAutonomousChoices();
        autonManager.displayChoices();
        configureButtonBindings();
    }

    /**
     * Use this method to define the command or command groups to be run at each event marker key. New
     * event markers can be created in PathPlanner.
     */
    private void eventAssignment() {
        eventMap.put("PreparePickup", new PreparePickup(false, Claw, Arm, Elevator, Intake));
        eventMap.put("Pickup", new Pickup(false, Claw, Arm, Elevator, Intake));
        eventMap.put("PlaceCubeMid", Place.Cube(0, Swerve, Limelight, Claw, Arm, Elevator, Intake));
        eventMap.put("PlaceConeMid", Place.Cone(0, Swerve, Limelight, Claw, Arm, Elevator, Intake));
        eventMap.put("PlaceCubeHigh", Place.Cube(1, Swerve, Limelight, Claw, Arm, Elevator, Intake));
        eventMap.put("PlaceLow", new AutonIntakeNoCurrentLimit(Intake).withTimeout(0.3));
        eventMap.put("Balance", new Balance(Swerve, Led));
        eventMap.put(
                "BalanceWithShoot", new BalanceWithShoot(Swerve, Led, Claw, Arm, Elevator, Intake));
    }

    // The RightPlacePickupPlaceBalance is : 1 foot from DriverStation blue line (x: 2.16), 6 inches
    // from Right wall (y: 0.76).
    /** Use this method to add Autonomous paths, displayed with {@link AutonManager} */
    private void addAutonomousChoices() {
        autonManager.addDefaultOption("Do Nothing.", new InstantCommand());

        for (String auton : autonChoices) {
            var trajectories = PathPlanner.loadPathGroup(auton, Constants.Auton.kPathConstr);
            Command autoPath = autoBuilder.fullAuto(trajectories);
            autonManager.addOption(auton, autoPath);
        }
    }

    public static boolean coneModeEnabled() {
        return isCone;
    }

    public static void enableConeMode(boolean value) {
        isCone = value;
    }

    public static double getAngleOffset() {
        return angleOffset;
    }

    public static void setAngleOffset(double offset) {
        angleOffset = offset;
    }

    public static State getCurrentState() {
        return currentState;
    }

    public static void setCurrentState(State state) {
        currentState = state;
    }

    public void autonResetGyro() {
        Swerve.zeroGyro(180);
    }

    private void configureButtonBindings() {
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.

        final int translationAxis = XboxController.Axis.kLeftY.value;
        final int strafeAxis = XboxController.Axis.kLeftX.value;
        final int rotationAxis = XboxController.Axis.kRightX.value;

        Swerve.setDefaultCommand(
                new TeleopSwerve(Swerve, driver, translationAxis, strafeAxis, rotationAxis));

        Elevator.setDefaultCommand(new DriveElevator(operator, Elevator));
        Arm.setDefaultCommand(new DriveArmElevator(operator, Arm));
        Claw.setDefaultCommand(new DriveClaw(operator, Claw));
        Intake.setDefaultCommand(new TeleopIntake(operator, Intake));

        // April Tag Mode
        driver.leftTrigger.onTrue(
                new InstantCommand(
                        () -> {
                            Limelight.setMode(1);
                            isCone = false;
                        }));

        // Reflective Tape Mode
        driver.rightTrigger.onTrue(
                new InstantCommand(
                        () -> {
                            Limelight.setMode(3);
                            isCone = true;
                        }));

        driver.buttonA.onTrue(new InstantCommand(Swerve::toggleSwerveMode));
        driver.buttonY.onTrue(new InstantCommand(Swerve::zeroGyro));
        driver.rightBumper.onTrue(new StowPosition(Elevator, Arm, Claw));
        driver.buttonStart.onTrue(Led.animationRun("VIVELAFRANCE", true));
        driver.buttonBack.onTrue(Led.animationRun("VIVELAFRANCE", false));

        operator.buttonA.onTrue(new StowPosition(Elevator, Arm, Claw));
        operator.buttonB.onTrue(new MidNodePosition(Elevator, Arm, Claw));
        operator.buttonY.onTrue(new HighNodePosition(Elevator, Arm, Claw));
        operator.buttonX.onTrue(new Position(Arm, Elevator, Claw));
        operator.dPadDown.onTrue(new IntakeFromGroundPosition(Elevator, Arm, Claw));
        operator.dPadUp.onTrue(new IntakeFromHumanPlayerPosition(Elevator, Arm, Claw));
        operator.dPadRight.onTrue(new IntakeFromSlidingHumanPlayerPosition(Elevator, Arm, Claw));
        operator.dPadLeft.onTrue(new ShootPiece(Intake, Elevator, Arm, Claw));
    }

    public Command selectedAuton() {
        isCone = false;
        return autonManager.getSelected();
    }

    public void idleLimelight() {
        Limelight.setPipeline(7);
        Limelight.setMode(0); // pipeline default
    }
}
