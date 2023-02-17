package frc.robot.commands.DriveParts;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MechanicalParts.IntakeSubsystem;
import frc.util.Controller;

public class setIntake extends CommandBase {
  private final IntakeSubsystem s_Intake;
  private final Controller controller;

  public setIntake(Controller controller, IntakeSubsystem s_Intake) {
    addRequirements(s_Intake);
    this.s_Intake = s_Intake;
    this.controller = controller;
  }

  @Override
  public void execute() {
    double forward = controller.getRightTrigger();
    double reverse = controller.getLeftTrigger();
    double rate = forward - reverse;
    s_Intake.intake(rate);
  }

  @Override
  public void end(boolean interrupted) {
    s_Intake.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
