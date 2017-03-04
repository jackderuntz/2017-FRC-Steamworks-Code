
package org.usfirst.frc.team4206.robot;


import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import org.usfirst.frc.team4206.robot.REVDigitBoard;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

/**
 * 2017 Team 4206 Java Code
 */

public class Robot extends SampleRobot implements PIDOutput {
	
    RobotDrive robotDrive;
    Joystick driver;
    
    CANTalon frontLeft = new CANTalon(2);
    CANTalon rearLeft = new CANTalon(4);
    CANTalon frontRight = new CANTalon(3);
    CANTalon rearRight = new CANTalon(5);
    CANTalon climbermaster = new CANTalon(6);
    CANTalon climberslave = new CANTalon (7);
    Spark shooter = new Spark(0);
    Joystick operator = new Joystick(1);
    Relay led = new Relay(0);
    
    
    AHRS ahrs;
    //PIDController turnController;
    double rotateToAngleRate;
    
    static final double kP = .1;
    static final double kI = 0;
    static final double kD = 2;
    static final double kF = 0;
    
    static final double kToleranceDegrees = 2.0f;
    
    double climbAccum;
    
    // The channel on the driver station that the joystick is connected to
    final int joystickChannel	= 0;

    public Robot() {
    	UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(360, 240);
        

    	
    	
    	//Mecanum Drive Train
        robotDrive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
    	robotDrive.setInvertedMotor(MotorType.kFrontRight, true);	// invert the right side motors
    	robotDrive.setInvertedMotor(MotorType.kRearRight, true);		// you may need to change or remove this to match your robot
        robotDrive.setExpiration(0.1);
        
        //Encoders for each Talon SRX
        frontLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        frontRight.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        rearLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        rearLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        
        //Sets 2nd climber TalonSRX to FOllower mode
        climberslave.changeControlMode(CANTalon.TalonControlMode.Follower);
        climberslave.set(climbermaster.getDeviceID());

        //Extra Stuff
        driver = new Joystick(joystickChannel);
        ahrs = new AHRS(Port.kMXP);
        
        //PID Controller for Rotate to Angle Mode
        /*
        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        */
      //  LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
        
        climbAccum = 0.0;
        
        
    }

    
    /**
     * Runs the motors with Mecanum drive.
     */
    public void operatorControl() {
        robotDrive.setSafetyEnabled(false);
        
        while (isOperatorControl() & isEnabled()) {
        	
         led.set(Relay.Value.kReverse);
         led.set(Relay.Value.kOn);
            
/*----------driver Dead Zone----------------------------------------------------*/
            double y = 0;
            double x = 0;
            double turn = 0;
            double climbx = 0;
            
            	
            	
            if (driver.getX()>0.15||driver.getX()<-0.15){
            	x=driver.getX();	
            }
            if (driver.getY()>0.15||driver.getY()<-0.15){
            	y=driver.getY();
            }
            if (driver.getRawAxis(4)>0.15||driver.getRawAxis(4)<-0.15){
            	turn=driver.getRawAxis(4);
            }
            if (operator.getRawAxis(1)>0.3||operator.getRawAxis(1)<-0.3){
            	climbx=operator.getRawAxis(1);
            }
/*----------Drive Train-------------------------------------------------------------*/        	
        	/*
        	boolean rotateToAngle = false;
            if ( operator.getRawButton(1)) {
                ahrs.reset();
            }
            switch (operator.getPOV()){
            case 45:
            	turnController.setSetpoint(45.0f);
            	rotateToAngle = true;
            	break;
            case 90:
            	turnController.setSetpoint(90.0f);
            	rotateToAngle = true;
            	break;
            case 135:
            	turnController.setSetpoint(135.0f);
            	rotateToAngle = true;
            	break;
            case 180:
            	turnController.setSetpoint(179.9f);
            	rotateToAngle = true;
            	break;
            case 225:
            	turnController.setSetpoint(-135.0f);
            	rotateToAngle = true;
            	break;
            case 270:
            	turnController.setSetpoint(-90.0f);
            	rotateToAngle = true;
            	break;
            case 315:
            	turnController.setSetpoint(-45.0f);
            	rotateToAngle = true;
            	break;
            default:
            	break;
            }
          
            double currentRotationRate;
            if ( rotateToAngle ) {
                turnController.enable();					//This switches the rotate control from the PID controller
                currentRotationRate = rotateToAngleRate;	//To the driver conroller's joystick
            } else {
                turnController.disable();
                currentRotationRate = turn;
            }
            */
            try {
            	//robotDrive.mecanumDrive_Cartesian(x, y, currentRotationRate,0);
            	robotDrive.mecanumDrive_Cartesian(x, y, driver.getRawAxis(5), 0);
            } catch( RuntimeException ex ) {
                DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
            }
            
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
            

/*----------Climber-----------------------------------------------------------------*/
            climbermaster.set(climbx);
            climbermaster.set(climbx);
            

            

/*----------Encoders----------------------------------------------------------------*/
            double EncRearLeftPos = rearLeft.getEncPosition();
            double EncRearRightSpeed = rearRight.getSpeed();
            double EncFrontLeftSpeed = frontLeft.getSpeed();
            double EncFrontRightSpeed = frontRight.getSpeed();
            double EncRearLeftSpeed = rearLeft.getSpeed();
            double FeetPerSecond = (((EncRearLeftSpeed/4096)*Math.PI*2)*3);
            SmartDashboard.putNumber("Rear Left Position", EncRearLeftPos);
            SmartDashboard.putNumber("Rear Left Speed", EncRearLeftSpeed);
            SmartDashboard.putNumber("Rear Right Speed", EncRearRightSpeed);
            SmartDashboard.putNumber("Front Left Speed", EncFrontLeftSpeed);
            SmartDashboard.putNumber("Front Right Speed", EncFrontRightSpeed);
            SmartDashboard.putNumber("Velocity Rear Left", FeetPerSecond);
            
/*----------Gyro--------------------------------------------------------------------*/
            SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
            SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
            SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
            SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
           
            
            Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles
        }
    }


	@Override
	public void pidWrite(double output) {
		//rotateToAngleRate = output;
	}

	public void test() {
			while (isTest() && isEnabled()) {
				LiveWindow.run();
				Timer.delay(0.1);
			}
	}
}
    