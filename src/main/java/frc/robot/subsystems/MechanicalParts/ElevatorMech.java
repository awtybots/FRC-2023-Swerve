package frc.robot.subsystems.MechanicalParts;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ElevatorMech extends Subsystem {
    void setHeight(double inches);

    boolean atTargetHeight();

    double heightInches();

    double distToSetpoint();

    void drive(double pct);

    void stop();

    void zeroHeightEncoder();
}
