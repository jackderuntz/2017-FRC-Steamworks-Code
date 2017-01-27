
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
    
    CANTalon frontLeft = new CANTalon(7);
    CANTalon rearLeft = new CANTalon(6);
    CANTalon frontRight = new CANTalon(5);
    CANTalon rearRight = new CANTalon(8);
    Spark shooter = new Spark(0);
    
    AHRS ahrs;
    PIDController turnController;
    double rotateToAngleRate;
    
    static final double kP = 0.01;
    static final double kI = 0.00;
    static final double kD = 0.00;
    static final double kF = 0.00;
    
    static final double kToleranceDegrees = 2.0f;
    
    double climbAccum;
    
    // The channel on the driver station that the joystick is connected to
    final int joystickChannel	= 0;

    public Robot() {
    	CameraServer.getInstance().startAutomaticCapture();
    	
        robotDrive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
    	robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);	// invert the left side motors
    	robotDrive.setInvertedMotor(MotorType.kRearLeft, true);		// you may need to change or remove this to match your robot
        robotDrive.setExpiration(0.1);
        
        frontLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        frontRight.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        rearLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        rearLeft.setFeedbackDevice(FeedbackDevice.QuadEncoder);

        controller = new Joystick(joystickChannel);
        ahrs = new AHRS(SPI.Port.kMXP);
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
                turnController.enable();
                currentRotationRate = rotateToAngleRate;
            } else {
                turnController.disable();
                currentRotationRate = controller.getRawAxis(4);
            }
            try {
                /* Use the joystick X axis for lateral movement,          */
                /* Y axis for forward movement, and the current           */
                /* calculated rotation rate (or joystick Z axis),         */
                /* depending upon whether "rotate to angle" is active.    */
                robotDrive.mecanumDrive_Cartesian(controller.getX(), controller.getY(), 
                                               currentRotationRate,0);
            } catch( RuntimeException ex ) {
                DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
            }
            
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
            robotDrive.mecanumDrive_Cartesian(controller.getX(), controller.getY(), controller.getRawAxis(4), ahrs.getAngle());
            
            climbAccum = controller.getRawAxis(3) + controller.getRawAxis(4);
            
            shooter.set(climbAccum);

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
    
}