// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import com.ctre.phoenix6.hardware.TalonFX;

/** Runs an Individual Real Module with all Motors as Neos */
public class ModuleIOSparkMax implements ModuleIO {
  private final TalonFX driveTalonFX;
  // private final CANSparkMax turnSparkMax;

  // private final RelativeEncoder turnRelativeEncoder;
  // private final CANcoder turnAbsoluteEncoder;

  // private final boolean isTurnMotorInverted = true;
  // private final double absoluteEncoderOffset;
  // private final int swerveModuleNumber;

  public ModuleIOSparkMax(int index) {
    driveTalonFX = new TalonFX(8);
  }

  @Override
  /**
   * updates the inputs to be actual values
   *
   * @param inputs from ModuleIOInputsAutoLogged
   */
  public void updateInputs(ModuleIOInputs inputs) {}
}
