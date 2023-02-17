package frc.robot.subsystems.MechanicalParts;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PistonSubsystem extends SubsystemBase {

    private final DoubleSolenoid piston;
    private boolean pistonState = false;

    public PistonSubsystem() {
        piston =
                new DoubleSolenoid(
                        PneumaticsModuleType.REVPH,
                        Constants.PistonConstants.kTraverseSolenoidF,
                        Constants.PistonConstants.kTraverseSolenoidR);
    }

    public void Open() {
        piston.set(Value.kForward);
        pistonState = false;
    }

    public void Close() {
        piston.set(Value.kReverse);
        pistonState = true;
    }
}
