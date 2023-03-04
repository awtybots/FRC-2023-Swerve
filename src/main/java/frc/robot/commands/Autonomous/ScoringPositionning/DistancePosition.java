package frc.robot.commands.Autonomous.ScoringPositionning;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.Swerve;

public class DistancePosition extends CommandBase {

    private final Swerve s_Swerve;

    // ! add to constants
    private double rotateThreshold = 3;
    private double driveThreshold = 3;
    private double rotateSpeed = 1;
    private double driveSpeed = 0.5;
    private double rotation;
    private Translation2d translation;
    private double offset;

    public DistancePosition(Swerve s_Swerve) {
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;

        this.rotation = 0;
        this.translation = new Translation2d(0, 0);

        this.offset = 7.5;
    }

    @Override
    public void execute() {
        s_Swerve.drive(new Translation2d(1, 0), 0, false);
    }

    // @Override
    public boolean isFinished() {
        return s_Swerve.getOutputCurrent() > 10;
    }
}
