// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

import monologue.Logged;

import static frc.robot.utilities.SparkConfigurator.getSparkMax;
import frc.robot.Constants.kIntake;
import frc.robot.utilities.SparkConfigurator.LogData;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import java.util.function.BooleanSupplier;
import java.util.Set;



public class IntakeRollers extends SubsystemBase implements Logged {
  /** Creates a new IntakeRollers. */
  public CANSparkMax intakeRollerMotor;
  public RelativeEncoder intakeRollerEncoder;
  public DigitalInput pieceCheck;




  public IntakeRollers() {
    intakeRollerMotor = 
        getSparkMax(
          kIntake.kRollers.rollerMotorID, 
          CANSparkLowLevel.MotorType.kBrushless, 
          false, 
          Set.of(), 
          Set.of(LogData.CURRENT, LogData.VOLTAGE, LogData.POSITION));

    intakeRollerMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    intakeRollerMotor.setInverted(kIntake.kRollers.invert);
    intakeRollerMotor.setSmartCurrentLimit(kIntake.kRollers.currentLimit);
    intakeRollerMotor.setOpenLoopRampRate(kIntake.kRollers.rampRate);
    intakeRollerMotor.setClosedLoopRampRate(kIntake.kRollers.rampRate); //Might need to remove
    intakeRollerMotor.burnFlash();

    pieceCheck = new DigitalInput(kIntake.kRollers.sensorChannel);
    intakeRollerEncoder = intakeRollerMotor.getEncoder();
  }


  public void setRollerSpeed(double speed){  //Not used
    intakeRollerMotor.set(speed);
  }

  public void setRollers(double voltage){
    intakeRollerMotor.setVoltage(voltage);
  }

  public BooleanSupplier getPieceCheck(){
    return () -> !pieceCheck.get();
  }

  public Command intake() {
    return 
      run(() -> setRollers(-kIntake.kRollers.intakeVoltage))
     .until(getPieceCheck())     //.until(getPieceCheck())
     .andThen(
          run(() -> setRollers(-kIntake.kRollers.intakeVoltage))
         .withTimeout(0.5))
     .finallyDo(() -> setRollers(0));
     //.withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
  }

  public Command rawIntake() {
    return 
      run(() -> setRollers(-kIntake.kRollers.intakeVoltage))
     .finallyDo(() -> setRollers(0));
  }

  public Command outtake() {
    return 
      run(() -> setRollers(-kIntake.kRollers.outtakeVoltage))
     .finallyDo(() -> setRollers(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
