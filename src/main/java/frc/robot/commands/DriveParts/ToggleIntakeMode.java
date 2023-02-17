package frc.robot.commands.DriveParts;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MechanicalParts.PistonSubsystem;

public class ToggleIntakeMode extends CommandBase {
  private final PistonSubsystem sPiston;
  private final boolean openTrue;

  public ToggleIntakeMode(PistonSubsystem sPiston, boolean openTrue) {
    addRequirements(sPiston);
    this.sPiston = sPiston;
    this.openTrue = openTrue;
  }

  @Override
  public void execute() {
    if (openTrue) {
      sPiston.Close();
    } else {
      sPiston.Open();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
