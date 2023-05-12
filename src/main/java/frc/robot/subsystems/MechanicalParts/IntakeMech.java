package frc.robot.subsystems.MechanicalParts;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IntakeMech extends Subsystem {
    void intake(double pct, boolean keep);

    void stopIntake();
}
