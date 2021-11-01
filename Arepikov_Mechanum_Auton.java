package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class Arepikov_Mechanum_Auton extends LinearOpMode{
    private Blinker     control_Hub, expansion_Hub_2;
    private BNO055IMU   c_imu, e_imu;
    private DcMotor     leftBack, leftFront, rightBack, rightFront;
    
    private double[]    motor = {0, 0, 0, 0};
    
    //actual angle is the angle read directly from the sensors
    //angle has the offset applied
    private double      actualAngle, angle;
    private double      offset;

    //setup land
    
    public void motorSetup()
    {
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightBack.setDirection(DcMotor.Direction.REVERSE);
        
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
    }
    
    public void gyroSetup()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        
        parameters.mode             = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit        = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled   = false;

        c_imu = hardwareMap.get(BNO055IMU.class, "imu");
        e_imu = hardwareMap.get(BNO055IMU.class, "imu2");

        c_imu.initialize(parameters);
        e_imu.initialize(parameters);
    }
    
    //::angle land
    
    public void getAngle()
    {
        Orientation c_angles = c_imu.getAngularOrientation(
        AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        
        Orientation e_angles = e_imu.getAngularOrientation(
        AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        angle = (c_angles.firstAngle + e_angles.firstAngle)/2;
        actualAngle = angle + offset;
    }
    
    public void offset()
    {
        offset = -angle;
    }
    
    //::other internal functions
    public void setPower()
    {
        rightBack.setPower(motor[0]/100);
        leftBack.setPower(motor[1]/100);
        leftFront.setPower(motor[2]/100);
        rightFront.setPower(motor[3]/100);
    }
    
    //sleep for ms milliseconds
    public void delay(long ms)
    {
        try {
            TimeUnit.MILLISECONDS.sleep(ms);
        }
        catch(InterruptedException e) {
            
        }
    }
    
    //motor access functions
    //use these!
    /* notes on all these functions
    every instance of power as a parameter expects inputs in the range of -100, 100.
    every instance of angle as a parameter expects inputs in the range of -180, 180.
    every instance of milliseconds as a parameter expects inputs in the range of 0, ∞
    
    */
    
    //move at the angle specified
    //0 is forward relative to the robot
    private void goAngle(double power, double angle, int milliseconds)
    {
        double robotAngle = (angle+90)*(Math.PI/180);
        
        motor[0] = power * Math.sin(robotAngle + (Math.PI/4));
        motor[1] = power * Math.sin(robotAngle - (Math.PI/4));
        motor[2] = power * Math.sin(robotAngle + (Math.PI/4));
        motor[3] = power * Math.sin(robotAngle - (Math.PI/4));
        
        setPower();
        delay(milliseconds);
    }
    
    //set the raw power of the motors
    //power is a list of parameters. 
    //ex: setRawPower(1000, 100, 100, 50, 75)
    //sets motor 0 to 100, motor 1 to 100, motor 2 to 50, and motor 3 to 75
    //right back, left back, left front, right back
    private void setRawPower(int milliseconds, double... power)
    {
        for (int i = 0; i<power.length && i<=4; i++)
        {
            motor[i] = power[i];
        }
        
        setPower();
        delay(milliseconds);
    }
    
    //stops the motors
    private void stopMotors()
    {
        motor[0] = 0;
        motor[1] = 0;
        motor[2] = 0;
        motor[3] = 0;
        setPower();
    }
    
    //turns the robot
    private void turn(double power, double turnAngle)
    {
        power = Math.abs(power);
        getAngle();
        double deltaAngle = actualAngle+turnAngle;
        while (Math.abs(deltaAngle) > 5)
        {
            opModeIsActive();
            if (deltaAngle > 0)
            {
                motor[0] = power;
                motor[1] = -power;
                motor[2] = -power;
                motor[3] = power;
            } else {
                motor[0] = -power;
                motor[1] = power;
                motor[2] = power;
                motor[3] = -power;
            }
            getAngle();
            setPower();
            deltaAngle = turnAngle-actualAngle;
        }
    }
    
    // the same as the above function, but with an optional accuracy parameter
    // accuracy is how many degrees +- the correct is acceptable
    // considering the above function works at all powers, 
    // this would mainly be used to dial in the accuracy tighter. 
    // if you're not careful the robot will skip over the accuracy and will spin forever
    // use at your own list.
    private void turn(double power, double accuracy, double turnAngle)
    {
        power = Math.abs(power);
        getAngle();
        double deltaAngle = actualAngle+turnAngle;
        while (Math.abs(deltaAngle) > accuracy)
        {
            opModeIsActive();
            if (deltaAngle > 0)
            {
                motor[0] = power;
                motor[1] = -power;
                motor[2] = -power;
                motor[3] = power;
            } else {
                motor[0] = -power;
                motor[1] = power;
                motor[2] = power;
                motor[3] = -power;
            }
            getAngle();
            setPower();
            deltaAngle = turnAngle-actualAngle;
        }
    }
    
    private void turnSingleMotor(int motorNumber, double power, int milliseconds) 
    {
        motor[motorNumber] = power;
        setPower();
        delay(milliseconds);
    }
    
    @Override
    public void runOpMode() {
        opModeIsActive();
        //put setup stuff here
        motorSetup();
        gyroSetup();
        getAngle();
        
        waitForStart();
        //code goes here
        goAngle(100, 45, 1000);
        
        //code ends here
        while(opModeIsActive())
        {
            stopMotors();
        }
    }
}

class color
{
    //
    static int green =      0x00ff00;
    
    //
    static int lightBlue =  0x00ffff;
    
    //
    static int darkBlue =   0x0000ff;
    
    //
    static int red =        0xff0000;
    
    //
    static int yellow =     0xffff00;
    
    //
    static int black =      0x000000;
    
    //
    static int white =      0xffffff;

    //static int lightGreen = 0xbada55;
    //static int darkGreen = 0x65535;
    //static int darkRed = 0x800000;
    //static int pink = 0xff80ed;
    //static int uglyYellow = 0x996515
}
