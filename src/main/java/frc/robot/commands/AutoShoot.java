package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends SequentialCommandGroup {
    public AutoShoot(Shooter shooter, Intake intake) {
        addCommands(
                new RunIntakeDouble(intake, 0.75),
                new RunShooterDouble(shooter, 0.60, 0.60));
    }
}
