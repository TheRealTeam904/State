/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.buttons.NetworkButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.ResetEncoder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.sensors.PigeonIMU;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  TalonSRX _LeftMaster = new TalonSRX(6);
  TalonSRX _LeftSlave = new TalonSRX(7);
  TalonSRX _LeftSlave1 = new TalonSRX(12);
  public static Encoder FourBarEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);

	TalonSRX _RightMaster = new TalonSRX(4);
  TalonSRX _RightSlave = new TalonSRX(5);
  TalonSRX _RightSlave1 = new TalonSRX(11);

	TalonSRX _Ball = new TalonSRX(8);
	TalonSRX _Climber = new TalonSRX(9);
	TalonSRX _FourBar = new TalonSRX(10);

	PigeonIMU _Pidgey = new PigeonIMU(10);

	Joystick _Driver = new Joystick(0);
  Joystick _Operator = new Joystick(1);

  double FourBar = -1 * _Operator.getRawAxis(5);
  double BallOutDriver = _Driver.getRawAxis(2);
  double BallOutOperator = _Operator.getRawAxis(2);
  double BallInOperator = _Operator.getRawAxis(3);

  Boolean BackCylinders;
  Boolean FullRotation;
  String Status;
  String ClimbStatus;

  Boolean Step1;
  Boolean Step2;
  Boolean Step3;
  Boolean Step4;
  Boolean Step5;
  Boolean Step6;
  Boolean Step7;
  Boolean Step8;

  Boolean StartClimb;

  Boolean FingerExtended;

  Timer ClimbTime = new Timer();

  public static Compressor Compressor = new Compressor(0);
  
  public static DoubleSolenoid Pivot = new DoubleSolenoid(0, 1);
	public static DoubleSolenoid.Value PivotUp = DoubleSolenoid.Value.kReverse;
  public static DoubleSolenoid.Value PivotDown = DoubleSolenoid.Value.kForward;
  
  boolean PivotedUp = true;

	public static DoubleSolenoid BackClimber = new DoubleSolenoid(2, 3);
	public static DoubleSolenoid.Value BackClimberClose = DoubleSolenoid.Value.kReverse;
	public static DoubleSolenoid.Value BackClimberExtended = DoubleSolenoid.Value.kForward;

  
	public static DoubleSolenoid Finger = new DoubleSolenoid(4, 5);
	public static DoubleSolenoid.Value FingerIn = DoubleSolenoid.Value.kReverse;
	public static DoubleSolenoid.Value FingerOut = DoubleSolenoid.Value.kForward;

  
  public double deadzone(double x) {
		if(x > 0.20)
			x = (x - 0.20) * 1.25;
		else if(x < -0.20)
			x = (x + 0.20) * 1.25;
		else
			x = 0;
		return x;
	}
	
	public double[] deadzone(double x, double y) {	
		return new double[] {(deadzone(x) * 0.75), (deadzone(y))};
  }
  
  
  public void drive(double turn, double forward) {
		double DriveRight = (forward - turn);
    double DriveLeft = (forward + turn );
    
    _LeftMaster.set(ControlMode.PercentOutput, DriveLeft);
    _RightMaster.set(ControlMode.PercentOutput, DriveRight);
	}
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    BackClimber.set(BackClimberClose);
    Pivot.set(DoubleSolenoid.Value.kReverse);
    Finger.set(FingerIn);

    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(1);
    FingerExtended = false;

    _LeftSlave.follow(_LeftMaster);
    _LeftSlave1.follow(_LeftMaster);
    _RightSlave.follow(_RightMaster);
    _RightSlave1.follow(_RightMaster);
   
    BackCylinders = false;
    FullRotation = false;

    Step1 = false;
    Step2 = false;
    Step3 = false;
    Step4 = false;
    Step5 = false;
    Step6 = false;
    Step7 = false;
    Step8 = false;

    StartClimb = false;

    /*NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(8);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("snapshot").setNumber(0);*/

    _LeftMaster.setNeutralMode(NeutralMode.Coast);
    _LeftSlave.setNeutralMode(NeutralMode.Coast);
    _LeftSlave1.setNeutralMode(NeutralMode.Coast);
    _RightMaster.setNeutralMode(NeutralMode.Coast);
    _RightSlave.setNeutralMode(NeutralMode.Coast);
    _RightSlave1.setNeutralMode(NeutralMode.Coast);
    _Climber.setNeutralMode(NeutralMode.Brake);

    _RightSlave.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    _RightSlave.setSelectedSensorPosition(0, 0, 30);

    _RightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    _RightMaster.setSelectedSensorPosition(0, 0, 30);

    _LeftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    _LeftMaster.setSelectedSensorPosition(0, 0, 30);

    _LeftMaster.setInverted(true);
    _LeftSlave.setInverted(true);
    _LeftSlave1.setInverted(true);
    _RightMaster.setInverted(false);
    _RightSlave.setInverted(false);
    _RightSlave1.setInverted(false);
    _FourBar.setInverted(false);

    _Climber.setInverted(true);
    
    Status = "initial";

    ClimbStatus = "inFrame";

		_LeftMaster.setSensorPhase(false);
    _RightMaster.setSensorPhase(false);

    _LeftMaster.configPeakOutputForward(+1.0, 30);
		_LeftMaster.configPeakOutputReverse(-1.0, 30);
		_RightMaster.configPeakOutputForward(+1.0, 30);
		_RightMaster.configPeakOutputReverse(-1.0, 30);

    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser); 

    SmartDashboard.putData("ResetEncoder", new ResetEncoder());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Scheduler.getInstance().run();

    SmartDashboard.putBoolean("BackCylinders", BackCylinders);
    SmartDashboard.putNumber("ClimberArm", _RightSlave.getSelectedSensorPosition());
    SmartDashboard.putBoolean("StartClimb", StartClimb);
    SmartDashboard.putString("ClimbStatus", ClimbStatus);

    SmartDashboard.putBoolean("Finger", FingerExtended);

    SmartDashboard.putNumber("leftMotor", _LeftMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("rightMotor", _RightMaster.getSelectedSensorPosition());
    SmartDashboard.putNumber("FourBarAmps", _FourBar.getOutputCurrent());    
    SmartDashboard.putNumber("FourBarEncoder", FourBarEncoder.getRaw());
    SmartDashboard.putString("Status", Status);
    SmartDashboard.putBoolean("LimitSwitchTop", _FourBar.getSensorCollection().isFwdLimitSwitchClosed());
    SmartDashboard.putBoolean("LimitSwitchBottom", _FourBar.getSensorCollection().isRevLimitSwitchClosed());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    double[] xy = deadzone( -1 * _Driver.getRawAxis(4), _Driver.getRawAxis(1));
    drive(xy[0], xy[1]);

    _Climber.set(ControlMode.PercentOutput, 0);
    _Ball.set(ControlMode.PercentOutput, .0);

    /*if(!StartClimb){
      if(_RightSlave.getSelectedSensorPosition() > 100){
        _Climber.set(ControlMode.PercentOutput, -0.1);
      }
      else if(_RightSlave.getSelectedSensorPosition() < -100){
        _Climber.set(ControlMode.PercentOutput, 0.1);
      }
      else{
        _Climber.set(ControlMode.PercentOutput, 0);
      }
    }*/

    if(ClimbStatus.matches("inFrame")){
      ClimbingArm(-100, 100);
    }
    
    //// Driver Controls
      if(_Driver.getRawButton(5)){
        FourBarEncoder.reset();
      }

      //// Operator Controls

      _FourBar.set(ControlMode.PercentOutput, -0.75 * deadzone(_Operator.getRawAxis(5)));
    

      if(_Operator.getRawButton(5)){
        FingerExtended = false; 
       }

      if(_Operator.getRawButton(8)){
        FingerExtended = true;
      }

      if(FingerExtended){
        Finger.set(FingerOut);
      }
      if(!FingerExtended){
        Finger.set(FingerIn);
      }

      if(_FourBar.getSensorCollection().isRevLimitSwitchClosed()){
        FourBarEncoder.reset();
      }
			
			if(_Operator.getRawAxis(2) >= 0.5){
				_Ball.set(ControlMode.PercentOutput, .4);
			}
			if(_Operator.getRawAxis(3) >= 0.5){
				_Ball.set(ControlMode.PercentOutput, -.5);
			}

      if(_Operator.getRawButton(1)){
        Status = "BallLow";
     }

     if(Status.matches("BallLow")){
      FourBarPresets(1800, 1870);
     }

     if(_Operator.getRawButton(2)){
      Status = "BallMiddle";
   }

   if(Status.matches("BallMiddle")){
    FourBarPresets(3550, 3620);
   }

   if(_Operator.getRawButton(4)){
    Status = "BallHigh";
 }

 if(Status.matches("BallHigh")){
  FourBarPresets(5000, 5050);
 }

 if(_Operator.getRawButton(1) && _Operator.getRawButton(7)){
  Status = "HatchLow";
}

if(Status.matches("HatchLow")){
FourBarPresets(550, 600);
}

if(_Operator.getRawButton(2) && _Operator.getRawButton(7)){
Status = "HatchMiddle";
}

if(Status.matches("HatchMiddle")){
FourBarPresets(2225, 2275);
}

if(_Operator.getRawButton(4) && _Operator.getRawButton(7)){
Status = "HatchHigh";
}

if(Status.matches("HatchHigh")){
FourBarPresets(4900, 5050);
}

      if((_Operator.getRawAxis(5) > 0.25) || (_Operator.getRawAxis(5) < -0.25)) {
        Status = "Live";
      } 



      if(_Operator.getRawButton(6)
      && FourBarEncoder.getRaw() > 250
      ){
          Pivot.set(DoubleSolenoid.Value.kReverse);
      }
      if(_Operator.getRawButton(3)
      && FourBarEncoder.getRaw() > 250
      ){
        Pivot.set(DoubleSolenoid.Value.kForward);
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    //LimelightDrive();

    double[] xy = deadzone( -1 * _Driver.getRawAxis(4), _Driver.getRawAxis(1));
    drive(xy[0], xy[1]);
    _Climber.set(ControlMode.PercentOutput, 0);
    _Ball.set(ControlMode.PercentOutput, 0);
    /*if(_Driver.getRawButton(6)){
      StartClimb = !StartClimb;
    }
    if(_Driver.getRawButton(6)){
      Step1 = true;
    }
      if(StartClimb){
        ClimberPreset();
      }
      else{
        Step1 = false;
        Step2 = false;
        Step3 = false;
        Step4 = false;
        Step5 = false;
        Step6 = false;
        Step7 = false;
        Step8 = false;
      }*/

      /*if(!StartClimb){
        if(_RightSlave.getSelectedSensorPosition() > 100){
          _Climber.set(ControlMode.PercentOutput, -0.1);
        }
        else if(_RightSlave.getSelectedSensorPosition() < -100){
          _Climber.set(ControlMode.PercentOutput, 0.1);
        }
        else{
          _Climber.set(ControlMode.PercentOutput, 0);
        }
      }*/
    
    //// Driver Controls
			if(_Driver.getRawButton(1)){
        _Climber.set(ControlMode.PercentOutput, 0.4);
        ClimbStatus = "Aiden";
      }

      if(ClimbStatus.matches("inFrame")){
        ClimbingArm(-100, 100);
      }

      if(_Driver.getRawButton(2)){
        _Climber.set(ControlMode.PercentOutput, -0.4);
      }
      if(_Driver.getRawButton(7)){
        _Climber.set(ControlMode.PercentOutput, 0.7);
      }
      if(_Driver.getRawButton(8)){
        _Climber.set(ControlMode.PercentOutput, -0.7);
      }
      if(_Driver.getRawButton(3)){
        BackClimber.set(BackClimberExtended);
      }
      if(_Driver.getRawButton(4)){
        BackClimber.set(BackClimberClose);
			}
			if(BallOutDriver >= 0.5){
        _Ball.set(ControlMode.PercentOutput, .5);
      }
      if(_Driver.getRawButton(5)){
        FourBarEncoder.reset();
      }

      //// Operator Controls

      _FourBar.set(ControlMode.PercentOutput, -0.75 * deadzone(_Operator.getRawAxis(5)));

      if(_Operator.getRawButton(5)){
        FingerExtended = false; 
       }

      if(_Operator.getRawButton(8)){
        FingerExtended = true;
      }


      if(FingerExtended){
        Finger.set(FingerOut);
      }
      if(!FingerExtended){
        Finger.set(FingerIn);
      }

      if(_FourBar.getSensorCollection().isRevLimitSwitchClosed()){
        FourBarEncoder.reset();
      }
			
			if(_Operator.getRawAxis(2) >= 0.5){
				_Ball.set(ControlMode.PercentOutput, .4);
      }
      else if(_Operator.getRawAxis(3) >= 0.5){
				_Ball.set(ControlMode.PercentOutput, -.5);
      }
      else {
        _Ball.set(ControlMode.PercentOutput, 0);
      }

      if(_Operator.getRawButton(1)){
        Status = "BallLow";
     }

     if(Status.matches("BallLow")){
      FourBarPresets(1800, 1870);
     }

     if(_Operator.getRawButton(2)){
      Status = "BallMiddle";
   }

   if(Status.matches("BallMiddle")){
    FourBarPresets(3550, 3620);
   }

   if(_Operator.getRawButton(4)){
    Status = "BallHigh";
 }

 if(Status.matches("BallHigh")){
  FourBarPresets(5000, 5050);
 }

 if(_Operator.getRawButton(1) && _Operator.getRawButton(7)){
  Status = "HatchLow";
}

if(Status.matches("HatchLow")){
FourBarPresets(550, 600);
}

if(_Operator.getRawButton(2) && _Operator.getRawButton(7)){
Status = "HatchMiddle";
}

if(Status.matches("HatchMiddle")){
FourBarPresets(2225, 2275);
}

if(_Operator.getRawButton(4) && _Operator.getRawButton(7)){
Status = "HatchHigh";
}

if(Status.matches("HatchHigh")){
FourBarPresets(4900, 5050);
}

if(Status.matches("InsideFrame")){
FourBarPresets(0, 0);
}


      if((_Operator.getRawAxis(5) > 0.25) || (_Operator.getRawAxis(5) < -0.25)) {
        Status = "Live";
      } 



      if(_Operator.getRawButton(6)
      && FourBarEncoder.getRaw() > 250
      ){
          Pivot.set(DoubleSolenoid.Value.kReverse);
      }
      if(_Operator.getRawButton(3)
      && FourBarEncoder.getRaw() > 250
      ){
        Pivot.set(DoubleSolenoid.Value.kForward);
    }
  }

  public void FourBarPresets(double TargetRangeLow, double TargetRangeTop){
    if ((FourBarEncoder.getRaw() < TargetRangeLow) && (Status != "Live")){
      _FourBar.set(ControlMode.PercentOutput, 0.5);
    }
    else if ((FourBarEncoder.getRaw() > TargetRangeTop) && (Status != "Live")){
      _FourBar.set(ControlMode.PercentOutput, -0.5);
    }
    else if (Status != "Live"){
      _FourBar.set(ControlMode.PercentOutput, 0);
    }
  }

  public void ClimbingArm(double RangeLow, double RangeTop){
    if((_RightSlave.getSelectedSensorPosition() > RangeTop) && (ClimbStatus != "Aiden")){
      _Climber.set(ControlMode.PercentOutput, -0.1);
    }
    else if((_RightSlave.getSelectedSensorPosition() < RangeLow) && (ClimbStatus != "Aiden")){
      _Climber.set(ControlMode.PercentOutput, 0.1);
    }
    else if (ClimbStatus != "Aiden"){
      _Climber.set(ControlMode.PercentOutput, 0);
    }
  }

/*
  public void ClimberPreset(){
    if(Step1 = true){
      Status = "BallMiddle";
    }
    if((FourBarEncoder.get() > 3500)){
      _RightSlave.setSelectedSensorPosition(0, 0, 30);
      Step2 = true;
    }
    if((_RightSlave.getSelectedSensorPosition() < 1750) && (Step2 = true)){
      _Climber.set(ControlMode.PercentOutput, 0.4);
      Step1 = false;
      Step3 = true;
    }
    if((_RightSlave.getSelectedSensorPosition() > 1750) && (FourBarEncoder.get() > 1) && (Step3 = true)){
      Status = "InsideFrame";
      _Climber.set(ControlMode.PercentOutput, 0);
      Step2 = false;
      Step4 = true;
    }
    if(_FourBar.getSensorCollection().isRevLimitSwitchClosed() && (Step4 = true)){
      BackClimber.set(BackClimberExtended);
      Step3 = false;
      Step5 = true;
      _LeftMaster.setSelectedSensorPosition(0, 0, 30);
      _RightMaster.setSelectedSensorPosition(0, 0, 30);
    }
    if((FourBarEncoder.get() < 10) && (_RightSlave.getSelectedSensorPosition() < 4000) && (_Climber.getOutputCurrent() < 15) && (Step5 = true)){
      _Climber.set(ControlMode.PercentOutput, 0.9);
      Step4 = false;
      Step6 = true;
    }
    if((_RightSlave.getSelectedSensorPosition() > 4000) && (Step6 = true)){
      _Climber.set(ControlMode.PercentOutput, 0);
      Step5 = false;
      Step7 = true;
    }
    if((_RightMaster.getSelectedSensorPosition() < 2000) && (_LeftMaster.getSelectedSensorPosition() < 2000) && (Step7 = true)){
      drive(0, -0.25);
      Step6 = false;
      Step8 = true;
    }
    if((_RightMaster.getSelectedSensorPosition() > 2000) && (_LeftMaster.getSelectedSensorPosition() > 2000) && (Step8 = true)){
      BackClimber.set(BackClimberClose);
      drive(0, 0);
    }
  }*/

  


  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}