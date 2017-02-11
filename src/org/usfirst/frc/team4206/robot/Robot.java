
package org.usfirst.frc.team4206.robot;


import edu.wpi.first.wpilibj.SampleRobot;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.CameraServer;

/**
 * 2017 Team 4206 Java Code
 */

public class Robot extends SampleRobot implements PIDOutput {
	
    RobotDrive robotDrive;
    Joystick controller;
    
    CANTalon frontLeft = new CANTalon(3);
    CANTalon rearLeft = new CANTalon(9);
    CANTalon frontRight = new CANTalon(2);
    CANTalon rearRight = new CANTalon(8);
    CANTalon climbermaster = new CANTalon(5);
    CANTalon climberslave = new CANTalon (6);
    Spark shooter = new Spark(0);
    Joystick ClimbStick = new Joystick(1);
    
    
    AHRS ahrs;
    PIDController turnController;
    double rotateToAngleRate;
    
    static final double kP = 15;
    static final double kI = 10;
    static final double kD = 10;
    static final double kF = 10;
    
    static final double kToleranceDegrees = 2.0f;
    
    double climbAccum;
    
    // The channel on the driver station that the joystick is connected to
    final int joystickChannel	= 0;

    public Robot() {
    	CameraServer.getInstance().startAutomaticCapture();
    	
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
        controller = new Joystick(joystickChannel);
        ahrs = new AHRS(SPI.Port.kMXP);
        
        //PID Controller for Rotate to Angle Mode
        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        
        LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
        
        climbAccum = 0.0;
        
        
    }

    /**
     * Runs the motors with Mecanum drive.
     */
    public void operatorControl() {
        robotDrive.setSafetyEnabled(false);
        
        while (isOperatorControl() & isEnabled()) {
/*----------Drive Train-------------------------------------------------------------*/        	
        	boolean rotateToAngle = false;
            if ( controller.getRawButton(1)) {
                ahrs.reset();
            }
            if ( controller.getRawButton(2)) {
                turnController.setSetpoint(0.0f);
                rotateToAngle = true;
            } else if ( controller.getRawButton(3)) {
                turnController.setSetpoint(45.0f);
                rotateToAngle = true;
            } else if ( controller.getRawButton(4)) {
                turnController.setSetpoint(179.9f);
                rotateToAngle = true;
            } else if ( controller.getRawButton(5)) {
                turnController.setSetpoint(-90.0f);
                rotateToAngle = true;
            }
            double currentRotationRate;
            if ( rotateToAngle ) {
                turnController.enable();					//This switches the rotate control from the PID controller
                currentRotationRate = rotateToAngleRate;	//To the driver conroller's joystick
            } else {
                turnController.disable();
                currentRotationRate = controller.getRawAxis(4);
            }
            try {
                robotDrive.mecanumDrive_Cartesian(controller.getX(), controller.getY(), 
                                               currentRotationRate,0);
            } catch( RuntimeException ex ) {
                DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
            }
            
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
            

/*----------Climber-----------------------------------------------------------------*/
            climbermaster.set(controller.getRawAxis(2));
            climbermaster.set(ClimbStick.getRawAxis(0));
            

            

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
		rotateToAngleRate = output;

	}

	public void test() {
			while (isTest() && isEnabled()) {
				LiveWindow.run();
				Timer.delay(0.1);
			}
	}
}
    