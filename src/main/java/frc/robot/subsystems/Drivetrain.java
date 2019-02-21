/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Drivetrain {

  private double ramp = 1.5;
  private double prevY = 0;

  private TalonSRX m_leftMaster, m_rightMaster;
  private VictorSPX m_leftFollowerOne, m_leftFollowerTwo, m_rightFollowerOne, m_rightFollowerTwo;

  public Drivetrain() {
    m_leftMaster = new TalonSRX(4);
    m_rightMaster = new TalonSRX(1);
    m_leftFollowerOne = new VictorSPX(5);
    m_leftFollowerTwo = new VictorSPX(6);
    m_rightFollowerOne = new VictorSPX(2);
    m_rightFollowerTwo = new VictorSPX(3);

    m_leftFollowerOne.follow(m_leftMaster);
    m_leftFollowerTwo.follow(m_leftMaster);
    m_rightFollowerOne.follow(m_rightMaster);
    m_rightFollowerTwo.follow(m_rightMaster);

    m_leftMaster.setInverted(false);
    m_leftFollowerOne.setInverted(false);
    m_leftFollowerTwo.setInverted(false);

    m_rightMaster.setInverted(true);
    m_rightFollowerOne.setInverted(true);
    m_rightFollowerTwo.setInverted(true);

    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 5);
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 5);

    m_leftMaster.configNominalOutputForward(0, 5);
    m_rightMaster.configNominalOutputReverse(0, 5);
    m_leftMaster.configPeakOutputForward(1, 5);
    m_rightMaster.configPeakOutputReverse(-1, 5);

    m_leftMaster.configPeakCurrentLimit(40, 5);
    m_leftMaster.configPeakCurrentDuration(100, 5);
    m_leftMaster.configContinuousCurrentLimit(30, 5);
    m_leftMaster.enableCurrentLimit(true);
    m_rightMaster.configPeakCurrentLimit(40, 5);
    m_rightMaster.configPeakCurrentDuration(100, 5);
    m_rightMaster.configContinuousCurrentLimit(30, 5);
    m_rightMaster.enableCurrentLimit(true);

    m_leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 40, 5);
    m_rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 40, 5);
  }

  public void arcadeDrive(double Y, double X) {

    double newY = Y;

    double maxIncrement = Robot.deltaTime * ramp;

    if (Y > prevY + maxIncrement){
      newY = prevY + maxIncrement;
    }

    m_leftMaster.set(ControlMode.Current, bound(1, -1, newY + X));
    m_rightMaster.set(ControlMode.Current, bound(1, -1, newY - X));

    prevY = newY;
  }

  public void tankDrive(double left, double right) {
    m_leftMaster.set(ControlMode.Current, bound(1, -1, left));
    m_rightMaster.set(ControlMode.Current, bound(1, -1, right));
  }

  public void teleopPeriodic() {
    
  }

  // Utility Functions

  public double getLeftEncoder() {
    return m_leftMaster.getSelectedSensorPosition(0);
  }

  public double getLeftVelocity() {
    return m_leftMaster.getSelectedSensorVelocity(0);
  }

  public double getRightEncoder() {
    return m_rightMaster.getSelectedSensorPosition(0);
  }

  public double getRightVelocity() {
    return m_rightMaster.getSelectedSensorVelocity(0);
  }

  public void setIdle(boolean brake) {
    if (brake) {
      m_leftMaster.setNeutralMode(NeutralMode.Brake);
      m_leftFollowerOne.setNeutralMode(NeutralMode.Brake);
      m_leftFollowerTwo.setNeutralMode(NeutralMode.Brake);
      m_rightMaster.setNeutralMode(NeutralMode.Brake);
      m_rightFollowerOne.setNeutralMode(NeutralMode.Brake);
      m_rightFollowerTwo.setNeutralMode(NeutralMode.Brake);
    } else {
      m_leftMaster.setNeutralMode(NeutralMode.Coast);
      m_leftFollowerOne.setNeutralMode(NeutralMode.Coast);
      m_leftFollowerTwo.setNeutralMode(NeutralMode.Coast);
      m_rightMaster.setNeutralMode(NeutralMode.Coast);
      m_rightFollowerOne.setNeutralMode(NeutralMode.Coast);
      m_rightFollowerTwo.setNeutralMode(NeutralMode.Coast);
    }
  }

  private double bound(double max, double min, double val) {
    if (val < min) {
      return min;
    } else if (val > max) {
      return max;
    } else {
      return val;
    }
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Drivetrain Left Pos", getLeftEncoder());
    SmartDashboard.putNumber("Drivetrain Right Pos", getRightEncoder());
    SmartDashboard.putNumber("Drivetrain Left Velo", getLeftVelocity());
    SmartDashboard.putNumber("Drivetrain Right Velo", getRightVelocity());
  }

}