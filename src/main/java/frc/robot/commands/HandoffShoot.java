// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.HandoffRollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HandoffShoot extends Command {
  /** Creates a new HandoffShoot. */

  private IntakeRollers m_intakeRollers;
  private HandoffRollers m_handoffRollers;
  private boolean noteIntaken;
  private boolean noteShootReady;

  public HandoffShoot(IntakeRollers intakeRollers, HandoffRollers handoffRollers) {
    m_intakeRollers = intakeRollers;
    m_handoffRollers = handoffRollers;
    noteIntaken = false;
    noteShootReady = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public Command handOffNote() {
    return Commands.deadline(m_handoffRollers.handoffIndexIntake(), m_intakeRollers.outtake())
          //.onlyIf(() -> noteIntaken == false)
          .andThen(() -> noteIntaken = true);
  }

  public Command indexToShooter() {
    return m_handoffRollers.handoffShooter()
          //.onlyIf(() -> noteIntaken == true)
          .andThen(() -> noteShootReady = true);
  }

  public Command feedtoShoot() {
    return m_handoffRollers.feedToShoot();
  }



}
