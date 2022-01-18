package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp

public class Arepikov_Mechanum_Teleop extends LinearOpMode{
    private DcMotor intake, leftBack, leftFront, rightBack, rightFront, lift;
    private Blinker control_Hub, expansion_Hub_2;
    private Servo claw, flipper;
    private BNO055IMU c_imu, e_imu;
    
    private DistanceSensor distance;
    private TouchSensor limit;
    private TouchSensor touchy;
    
    private double[] motor = {0, 0, 0, 0, 0, 0};
    
    private double angle, actualAngle, offset;
    
    private boolean headless;
    
    long headlessWait = getTime();
    
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
        
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //intake.setDirection(DcMotor.Direction.REVERSE);
        
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lift.setDirection(DcMotor.Direction.REVERSE);
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
    
    public void miscSetup()
    {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        
        //mag = hardwareMap.get(TouchSensor.class, "limit");
        //touch = hardwareMap.get(TouchSensor.class, "touchy");
        //ultra = hardwareMap.get(DistanceSensor.class, "distance");
        
        claw = hardwareMap.get(Servo.class, "claw");
        flipper = hardwareMap.get(Servo.class, "flipper");
    }
    
    //angle time
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
        rightBack.setPower(motor[0]);
        leftBack.setPower(motor[1]);
        leftFront.setPower(motor[2]);
        rightFront.setPower(motor[3]);
        
        intake.setPower(motor[4]);
        lift.setPower(motor[5]);
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
    
    //milliseconds
    private static long getTime()
    {
        return (System.nanoTime()) / 1000000;
    }
    
    //sets the color of the built in leds 
    public void light(int color)
    {
        control_Hub.setConstant(color);
        expansion_Hub_2.setConstant(color);
        
    }

    @Override
    public void runOpMode() {
        motorSetup();
        gyroSetup();
        miscSetup();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        waitForStart();
        while(opModeIsActive())
        {
            generalPower();
            getAngle();
            
            if (gamepad1.b)
            {
                returnToForward();
            } else if (gamepad1.y)
            {
                offset();
            } else if (gamepad1.x)
            {
                if ((getTime()) - headlessWait >= 500)
                {
                    headless = !headless;
                    headlessWait = getTime();
                }
            } 
            
            telemetry.addData("Angle", actualAngle);
            telemetry.addData("Offset", offset);
            telemetry.update();
            
            setPower();
        }
    }
    
    void servoPower()
    {
        claw.setPosition((gamepad2.right_stick_x + 1) / 2);
        flipper.setPosition((gamepad2.left_stick_x + 1) / 2);
    }
    
    void generalPower()
    {
        double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double rightX = gamepad1.right_stick_x;
        
        if (headless)
        {
            robotAngle += actualAngle*(Math.PI/180);
            light(color.red);
        } else {
            light(color.black);
        }
        
        servoPower();

        motor[0] = r * Math.sin(robotAngle - (Math.PI/4)) + rightX;
        motor[1] = r * Math.sin(robotAngle + (Math.PI/4)) - rightX;
        motor[2] = r * Math.sin(robotAngle - (Math.PI/4)) - rightX;
        motor[3] = r * Math.sin(robotAngle + (Math.PI/4)) + rightX;
        
        motor[4] = gamepad2.right_stick_y;
        motor[5] = gamepad2.left_stick_y;
    }
    
    private void returnToForward()
    {
        //if within 10 degrees
        if (Math.abs(actualAngle) <= 20)
        {
            while (Math.abs(actualAngle) >= 2)
            {
                light(color.lightBlue);
                opModeIsActive();
                
                if (actualAngle <= 0)
                {
                    motor[0] = -0.1;
                    motor[1] = 0.1;
                    motor[2] = 0.1;
                    motor[3] = -0.1;
                } else if (actualAngle >= 0) {
                    motor[0] = 0.1;
                    motor[1] = -0.1;
                    motor[2] = -0.1;
                    motor[3] = 0.1;
                }
    
                setPower();
                getAngle();
                telemetry.addLine("Precisely Homing...");
                telemetry.addData("Actual Angle", actualAngle);
                telemetry.update();
            }
            return;
        }
        
        //if not within 10 degrees at start
        while (Math.abs(actualAngle) >= 20)
        {
            light(color.darkBlue);;
            opModeIsActive();
            
            if (actualAngle <= 0)
            {
                motor[0] = -0.75;
                motor[1] = 0.75;
                motor[2] = 0.75;
                motor[3] = -0.75;
            } else if (actualAngle >= 0) {
                motor[0] = 0.75;
                motor[1] = -0.75;
                motor[2] = -0.75;
                motor[3] = 0.75;
            }

            setPower();
            getAngle();
            telemetry.addLine("Homing...");
            telemetry.addData("Actual Angle", actualAngle);
            telemetry.update();
        }
    }
}