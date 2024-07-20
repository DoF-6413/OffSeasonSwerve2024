package frc.robot.Commands.Auto.DeadReckons;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.drive.Drive;

public class RightAuto extends SequentialCommandGroup {

  public RightAuto(double speed, Drive drive, double time, boolean leftOrRight) {
    if (leftOrRight == true) {
      addCommands(
          Commands.runOnce(
              () -> {
                drive.setRaw(speed, 0, 0);
              },
              drive),
          new WaitCommand(time),
          Commands.runOnce(
              () -> {
                drive.setRaw(0, 0, 0);
              },
              drive));
    } else {
      addCommands(
          Commands.runOnce(
              () -> {
                drive.setRaw(-speed, 0, 0);
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
}
