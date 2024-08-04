// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.DriveConstants;
import frc.robot.Subsystems.gyro.Gyro;

public class PoseEstimator extends SubsystemBase {
  private Drive drive;
  private Gyro gyro;
  private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  private Field2d field2d;
  public static Vector<N3> stateStandardDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /** Creates a new PoseEstimator. */
  public PoseEstimator(Drive drive, Gyro gyro) {
    field2d = new Field2d();
    SmartDashboard.putData(field2d);
    this.drive = drive;
    this.gyro = gyro;
    swerveDrivePoseEstimator =
        new SwerveDrivePoseEstimator(
            new SwerveDriveKinematics(DriveConstants.getModuleTranslations()),
            gyro.getYaw(),
            drive.getSwerveModulePositions(),
            new Pose2d(new Translation2d(), new Rotation2d()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    field2d.setRobotPose(getCurrentPose2d());
    swerveDrivePoseEstimator.updateWithTime(
        Timer.getFPGATimestamp(), drive.getRotation(), drive.getSwerveModulePositions());
  }

  public Pose2d getCurrentPose2d() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d currentPose2d) {
    swerveDrivePoseEstimator.resetPosition(
        gyro.getAngle(), drive.getSwerveModulePositions(), currentPose2d);
  }

  public Rotation2d getRotation() {
    return swerveDrivePoseEstimator.getEstimatedPosition().getRotation();
  }
}
