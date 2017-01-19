
package org.usfirst.frc.team4206.robot;


import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Spark;
import com.ctre.CANTalon;

/**
 * This is a demo program showing how to use Mecanum control with the RobotDrive class.
 */

public class Robot extends SampleRobot {
	
    RobotDrive robotDrive;
    Joystick controller;
    
    CANTalon frontLeft = new CANTalon(7);
    CANTalon rearLeft = new CANTalon(8);
    CANTalon frontRight = new CANTalon(5);
    CANTalon rearRight = new CANTalon(6);
    Spark shooter = new Spark(0);
    
    ADXRS450_Gyro gyro;
    
    // The channel on the driver station that the joystick is connected to
    final int joystickChannel	= 0;

    public Robot() {
        robotDrive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
    	robotDrive.setInvertedMotor(MotorType.kFrontLeft, true);	// invert the left side motors
    	robotDrive.setInvertedMotor(MotorType.kRearLeft, true);		// you may need to change or remove this to match your robot
        robotDrive.setExpiration(0.1);

        controller = new Joystick(joystickChannel);
        
        gyro = new ADXRS450_Gyro();
    }
        

    /**
     * Runs the motors with Mecanum drive.
     */
    public void operatorControl() {
        robotDrive.setSafetyEnabled(false);
        gyro.calibrate();
        double angle = 0.0;
        while (isOperatorControl() & isEnabled()) {
        	angle = gyro.getAngle();
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
            robotDrive.mecanumDrive_Cartesian(controller.getX(), controller.getY(), controller.getRawAxis(4), angle);
            System.out.println(angle);
            
            shooter.set(controller.getRawAxis(3));
            
            Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles
        }
    }
    
}