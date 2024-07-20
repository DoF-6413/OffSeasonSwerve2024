package frc.robot.Commands.Auto.DeadReckons;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.drive.Drive;

public class LeaveAuto extends SequentialCommandGroup {

  public LeaveAuto(double speed, Drive drive, double time) {

    addCommands(
        Commands.runOnce(
            () -> {
              drive.setRaw(0, speed, 0);
            },
            drive),
        new WaitCommand(time),
        Commands.runOnce(
            () -> {
              drive.setRaw(0, 0, 0);
            },
            drive));
  }
}
