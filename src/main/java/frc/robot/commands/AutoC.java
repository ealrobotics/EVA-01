package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoC extends ParallelCommandGroup {
  public AutoC(Drivetrain drivetrain, Shooter shooter, Intake intake) {
    addCommands(
        new AutoDrive(drivetrain),
        new AutoShoot(shooter, intake));
  }
}
