/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Elevator {

  public double kP = 0.00003, kI = 0, kD = 0, kFF = 0;

  private CANSparkMax m_master, m_follower;
  private CANEncoder m_encoder;

  private CANPIDController m_pidController;
  private CANDigitalInput m_limitSwitchTop;
  private CANDigitalInput m_limitSwitchBottom;

  private double rampAccel = 1.66;

  public Elevator() {
    m_master = new CANSparkMax(42, MotorType.kBrushless);
    m_follower = new CANSparkMax(43, MotorType.kBrushless);

    m_master.setInverted(true);
    m_follower.setInverted(true);

    m_follower.follow(m_master, true);

    m_encoder = m_master.getEncoder();
    m_encoder.setPositionConversionFactor(42);

    m_limitSwitchTop = new CANDigitalInput(m_master, LimitSwitch.kReverse, LimitSwitchPolarity.kNormallyOpen);
    m_limitSwitchTop.enableLimitSwitch(true);
    
    m_limitSwitchBottom= new CANDigitalInput(m_master, LimitSwitch.kForward , LimitSwitchPolarity.kNormallyOpen);
    m_limitSwitchBottom.enableLimitSwitch(true);

    m_pidController = m_master.getPIDController();
    m_pidController.setOutputRange(-0.7, 0.7);
    m_pidController.setP(0.0);
    m_pidController.setI(0.0);
    m_pidController.setD(0.0);
    m_pidController.setFF(0.0);

    SmartDashboard.putNumber("Elevator Pid P", kP);
    SmartDashboard.putNumber("Elevator Pid I", kI);
    SmartDashboard.putNumber("Elevator Pid D", kD);
    SmartDashboard.putNumber("Elevator Pid FF", kFF);
  }

  public void teleopPeriodic() {
    
  }

}
