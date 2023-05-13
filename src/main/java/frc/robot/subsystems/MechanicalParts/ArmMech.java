package frc.robot.subsystems.MechanicalParts;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ArmMech extends Subsystem {
    void setExtent(double ticks);

    double getTickError();

    boolean atTargetExtent();

    void drive(double pct);

    void stop();
}
