package frc.robot.commands.Positions.Nodes;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class HighNodePosition extends CommandBase {

  private final ElevatorSubsystem s_elevator;
  private final ArmSubsystem s_arm;
  private final ClawSubsystem s_claw;

  public HighNodePosition(
      ElevatorSubsystem s_elevatorSubsystem,
      ArmSubsystem s_arArmSubsystem,
      ClawSubsystem s_ClawSubsystem) {
    addRequirements(s_elevatorSubsystem, s_arArmSubsystem, s_ClawSubsystem);
    this.s_elevator = s_elevatorSubsystem;
    this.s_arm = s_arArmSubsystem;
    this.s_claw = s_ClawSubsystem;
  }

  @Override
  public void execute() {
    s_elevator.setHeight(Constants.Position.Nodes.HighNodePosition.ElevatorPosition);
    s_arm.setRotation(Constants.Position.Nodes.HighNodePosition.ArmPosition);
    s_claw.setRotation(Constants.Position.Nodes.HighNodePosition.ClawPosition);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
