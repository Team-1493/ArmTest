
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  TalonFX armMotor=new TalonFX(9);
  CANCoder enc=new CANCoder(9);
  Joystick joy = new Joystick(0);
  ArmFeedforward feedFwd = new ArmFeedforward(0, 0, 0);
  double absPos,relPos;
  double armkP=0,armkD=0,armkG=0,armKs=0;
  double pos1=5,pos2=25;
  double angle;

  @Override
  public void robotInit() {
    enc.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    enc.setPositionToAbsolute(20);
    armMotor.configFactoryDefault();
    armMotor.setNeutralMode(NeutralMode.Coast);
    armMotor.configRemoteFeedbackFilter(enc, 0);
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0  , 0, 20);
    StatorCurrentLimitConfiguration currentConfig = 
        new StatorCurrentLimitConfiguration(true, 20, 25, 1);
    //armMotor.configGetStatorCurrentLimit(currentConfig)  ;
    armMotor.config_kP(0,armkP);
    armMotor.config_kD(0,armkD);
    armMotor.configMotionAcceleration(1);
    armMotor.configMotionCruiseVelocity(1);
  



    SmartDashboard.putNumber("arm kP", 0.0);
    SmartDashboard.putNumber("arm kD", 0.0);
    SmartDashboard.putNumber("arm kG", 0.0);
    SmartDashboard.putNumber("arm kS", 0.0);
    SmartDashboard.putNumber("arm kV", 0.0);
    SmartDashboard.putNumber("arm kA", 0.0);
    SmartDashboard.putNumber("arm pos1", 5);
    SmartDashboard.putNumber("arm pos2", 20);
    SmartDashboard.putNumber("arm MMvel", 0);
    SmartDashboard.putNumber("arm MMacc", 0);


  }

  @Override
  public void robotPeriodic() {
    
    SmartDashboard.putNumber("arm relPosMotor", armMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("arm relPosEnc", enc.getPosition());
    SmartDashboard.putNumber("arm absPosEnc", enc.getAbsolutePosition());
    SmartDashboard.putNumber("arm current", armMotor.getStatorCurrent());
    SmartDashboard.putNumber("arm current", armMotor.getStatorCurrent());
    SmartDashboard.putNumber("arm CLE", armMotor.getClosedLoopError());  
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    if(joy.getRawButton(1)){
//      armMotor.set(ControlMode.MotionMagic,pos1,
//      DemandType.ArbitraryFeedForward,feedFwd.calculate(angle-90, 0));
    }
    else if(joy.getRawButton(2)){
//      armMotor.set(ControlMode.MotionMagic,pos2,
//      DemandType.ArbitraryFeedForward,feedFwd.calculate(angle-90, 0));
    }
    else if(joy.getRawButton(4)){
      updateConstants();
    }
    else{
      double stick = joy.getRawAxis(0);    
//        armMotor.set(ControlMode.PercentOutput, 0.25*stick);
    }
  }



  public void updateConstants(){
    feedFwd=new ArmFeedforward(
        SmartDashboard.getNumber("arm kS",0),
        SmartDashboard.getNumber("arm kG",0),
        SmartDashboard.getNumber("arm kV",0),
        SmartDashboard.getNumber("arm kA",0)
        );

    armMotor.config_kP(0,SmartDashboard.getNumber("arm kP", 0));
    armMotor.config_kD(0,SmartDashboard.getNumber("arm kD", 0));

    armMotor.configMotionAcceleration(SmartDashboard.getNumber("arm MMacc", 0));
    armMotor.configMotionAcceleration(SmartDashboard.getNumber("arm MMVel", 0));   
    
    pos1=SmartDashboard.getNumber("arm pos1", 0);
    pos2=SmartDashboard.getNumber("arm pos2", 0);

  }
 
}
