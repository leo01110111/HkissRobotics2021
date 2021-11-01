/*
Hello!

This is the start of me commenting this code. Currently, all comments are made
by arepikov.

Current and Past code authors are:
arepikov

Please add your name to the list once you feel that you have made enough edits
to call yourself an author.

DOCUMENTATION FOR FTC ONBOT JAVA PACKAGE CAN BE FOUND AT:
https://ftctechnh.github.io/ftc_app/doc/javadoc/index.html

Personal notes

arepikov:
Hello! I am the "original" author of this file. Most changes from the original
example source were made by me, including adding a shit ton of code (4k to 
whatever its at right now (28k?))
If you hate my comments, remember that they're better than nothing. And then you
can hate me. That's cool with me.
Have fun!
*/

//importing packages
//My best guess for what each package does is included below
//some packages are just kinda there? idk

//overall package?
package org.firstinspires.ftc.teamcode;

//telling the robot what kind of program this is (TeleOp (not autonomous))
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//The same thing as above? but without the capability for loops? idk
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//allowing access to the onboard LED.
import com.qualcomm.robotcore.hardware.Blinker;
//allows disabling the code by typing @Disabled. Code like this will not be
//visible on the phone as an option.
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

//gives access to the built in IMU (gyroscope)
import com.qualcomm.hardware.bosch.BNO055IMU;
//allows access to the DC motors. Very useful for moving.
import com.qualcomm.robotcore.hardware.DcMotor;

//Ok, I'm not going to go over every single one of these
//Basically they all just allow access to different things about the IMU
//Don't think we use most of these. So take them out? IDK I'm lazy at the moment
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

//Can be used for the timer, and probably should, but I can't figure out how to
//work with it. So there's another thing to do at some point.
import com.qualcomm.robotcore.util.ElapsedTime;

//used for ArrayLists, which are like arrays but better
import java.util.ArrayList;
//for sleeping accurately
import java.util.concurrent.TimeUnit;

//access android Text to Speech
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

//If there is a @Disabled two lines down(ish), this program will not be added to
//the robot. Add it (make sure to caps the D) to remove this program from the
//list of runnable programs.

@TeleOp

//Not sure why it's linear but nothing else works (it fails to compile)
//where the fun starts
public class MechanumDrive extends LinearOpMode
{
    //this entire section of code (up until :INIT) is used for defining
    //variables.

    //location tag
    //::DEFINE


    //Alright, it's definition time.
    //I'll do my best but I have no clue what some of this does (yes i know)
    //WHY ARE DEFINITIONS SO F***** COMPLICATED!?

    //used later to initialize the led on the control hub.
    Blinker         blinkLight;
    //used later to initialize the motors. Motor names are what you expect
    DcMotor         leftBack, rightBack, rightFrontal, leftFrontal, intake;
    //used later to initialize the IMU
    BNO055IMU       imu;
    //Honestly no clue. Something to do with the gyroscope/IMU
    //actually used though, don't delete
    Orientation     lastAngles = new Orientation();
    
    /*
    three angle measurements
    No clue what globalAngle is. 
    Angle is cumulative and never used (so far)
    actual Angle is (usually) from -180 to 180. Used the most.
    */
    double          globalAngle, angle, actualAngle;
    
    //used to set the offset angle
    double          offset=0;

    //ok this is actually used
    //it's used for the braking function
    double          bumperOffset;
    //counts how many times the robot has gone through the main loop
    int             loopCount1;
    //used in some timer fuckery. It's explained where this is used
    long            headlessWait;
    //the motor powers. An array from 0-4 for each of the 4 motors
    double []       motor = new double[5];
    
    //More motor stuff. r is the power (see trig (x, y, r)).
    //robotAngle is actualy the angle of the left joystick
    //rightX is just used another name for gamepad1.right_stick_x
    double          r, robotAngle, rightX;

    //used to keep track of if headless mode is on or not.
    boolean         headless=false;

    //records start time in nanoseconds. Should at some point be replaced with
    //util.ElapsedTime but hey this works.
    //Used in the actual function (which returns seconds since program 
    //start)
    static long     start = System.nanoTime();
    
    //setting the main loop on a consistant time
    long            mainLoopStart;

    //an array of all of the previous motor powers
    //will at some point be used to return to a home point
    //explained wherever that's implemented
    //if it's not have fun
    static          ArrayList<Double> arrayV1 = new ArrayList<Double>();
    static          ArrayList<Double> arrayV2 = new ArrayList<Double>();
    static          ArrayList<Double> arrayV3 = new ArrayList<Double>();
    static          ArrayList<Double> arrayV4 = new ArrayList<Double>();


    /*
    A mapping for simple color names
    accessed by using color.color_name
    where color_name is the name of the color listed below
    new entries can be added by using the color's hex code
    commented out entries are too similar and cannot be distinuished
    add them back in at your own risk
    I don't care.
    */
    public static class color
    {
        //return to home
        static int green =      0x00ff00;
        
        //used for homing angle (accurate)
        static int lightBlue =  0x00ffff;
        
        //used for homing angle (regular)
        static int darkBlue =   0x0000ff;
        
        //headless
        static int red =        0xff0000;
        
        //UNUSED
        static int yellow =     0xffff00;
        
        //General purpose
        static int black =      0x000000;
        
        //UNUSED
        static int white =      0xffffff;

        //static int lightGreen = 0xbada55;
        //static int darkGreen = 0x65535;
        //static int darkRed = 0x800000;
        //static int pink = 0xff80ed;
        //static int uglyYellow = 0x996515
    }
    

    @Override
    public void runOpMode() {

        //this is the code that is run when the "init" button is pressed on the
        //phone. Most stuff should be here and not in :DEFINE

        //location tag
        //::INIT
        
        //motor stuff inside here
        {
            //set up motor stuff
            //there is *so much stuff*
            //but that's how it needs to be done
            //everything is repeated 4 times, once for each motor (usually)
            //the brackets are just so that I can minimize this huge block
            
            //Give motors names
            //pretty sure these are set up on the phone
            leftBack = hardwareMap.get(DcMotor.class, "leftBack");
            rightFrontal = hardwareMap.get(DcMotor.class, "rightFront");
            leftFrontal = hardwareMap.get(DcMotor.class, "leftFront");
            rightBack = hardwareMap.get(DcMotor.class, "rightBack");
            
            intake = hardwareMap.get(DcMotor.class, "intake");
    
            //set motors to speed mode
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
            //set motors to stop at 0 power instead of coasting
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFrontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            
            /*
            set two motors to go backwards
            so funny story.
            This actually works now. It used to be really stupid, but then I 
            spent some time and fixed it. It makes sense now. 
            So now {1, 1, 1, 1} makes the robot go forward. 
            */
            rightBack.setDirection(DcMotor.Direction.REVERSE);
            rightFrontal.setDirection(DcMotor.Direction.REVERSE);
    
            //OK the motor stuff is (finally) done
        }


        //set up the the LED on the control hub
        blinkLight = hardwareMap.get(Blinker.class, "Control Hub");
        
        //clear the motor power arrays
        arrayV1.clear();
        arrayV2.clear();
        arrayV3.clear();
        arrayV4.clear();

        //gyroscope stuff inside
        {
            //set up gyroscope stuff
            //set up the parameters varaible??
            //I have no clue what most of this does
            //It's just copy and pasted.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.mode             = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit        = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled   = false;
    
            //set up the IMU (kinda)
            imu = hardwareMap.get(BNO055IMU.class, "imu");
    
            //actually set up the IMU
            imu.initialize(parameters);
        }
        
        /*
        Send telemetry to phone
        side note: this is the correct format for sending telemetry
        so take a look at this if you're learning
        also the imu.getCalibrationStatus thing is just getting how
        calibrated the various sensors of the IMU are.
        Not really something to worry about.
        */
        telemetry.addData("Calibration", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //Waits for the "play" button (actually a triangle) to get pressed on
        //the phone
        waitForStart();
        
        //Runs until the code is killed by pressing stop on the phone.
        while (opModeIsActive()) {
            //where the ACTUAL fun begins
            //this is the runtime code

            //location tag
            //::

            //used to tell the robot that there will be loops
            //there will be loops.
            opModeIsActive();
            
            //set the time the main loop started
            mainLoopStart = getTime();
            
            //get the various angles
            //see the actual function for more details
            //mainly used to get actualAngle
            getAngle();

            //set v1-4 to useful things
            //also their final things if nothing changes them
            //again see function for more details.
            generalPower();

            /*
            so this if statement stack is used to define many of the
            interesting subroutines that the robot has.
            It looks at the gamepad and just goes through the stack looking
            for for the first thing that is pressed
            done this way to prevent the robot from trying to do two things
            at once.
            Go see the functions for detailed explainations of what everything
            does.
            */

            //return to start angle
            if (gamepad1.b == true)
            {
                returnToForward();
            }//set start angle
            else if (gamepad1.y == true)
            {
                offset();
            } //headless mode
            else if (gamepad1.x == true)
            {
                /*
                Ok this is really weird
                basically check if it has been more than one second since the
                last time that headless mode was changed.
                if this isn't here then it turns it on and off many times on
                even the shortest button press
                note that the time is in milliseconds
                HeadlessWait is the last time that headless mode status  changed
                */
                if ((getTime()) - headlessWait >= 1000)
                {
                    headless = switchInput(headless);
                    headlessWait = getTime();
                }
            } //used for setting a new home location (return)
            else if (gamepad1.left_bumper)
            {
                setNew();
            } //going to be used for returning home
            else if (gamepad1.right_bumper)
            {
                returnHome();
            }
            

            //actually set the motors to whatever power they need to be set to
            //so just change v1-4 before this point and this will take care of
            //the rest
            setPower();

            //just preventing weird issues caused by overflowing loopcounts
            //currently 100,000,000 <- please update if changed
            if (loopCount1 == 100000000)
            {
                loopCount1 = 0;
            }
            
            //Set the color of the LED
            if (headless)
            {
                blinkLight.setConstant(color.red);
            } else {
                blinkLight.setConstant(color.black);
            }
            
            /*
            So just a shit ton of telemetry
            the commented stuff is useful sometimes.
            add it back in if you need it
            If you want, change that one in the if statement to something else 
            that will make the telemetry only run once every _number_ loops.
            */
            if (loopCount1 % 1 == 0) {
                telemetry.addData("Status", "Running");
                telemetry.addData("Calibration", imu.getCalibrationStatus().toString());
                telemetry.addData("Actual Angle", actualAngle);
                telemetry.addData("Headless Mode Active?", headless);
                telemetry.addData("time", time);
                telemetry.addData("Length of Homing path", arrayV1.size());
                //telemetry.addData("Android Angle", AndroidGyroscope.startListening());

                /*
                telemetry.addData("Left Front", v1);
                telemetry.addData("Right Front", v2);
                telemetry.addData("Left Back", v3);
                telemetry.addData("Right Front", v4);
                telemetry.addData("joyLX", gamepad1.left_stick_x);
                telemetry.addData("joyRX", gamepad1.right_stick_x);
                telemetry.addData("joyLY", gamepad1.left_stick_y);
                telemetry.addData("joyRY", gamepad1.right_stick_y);
                telemetry.addData("robotAngle", robotAngle);
                telemetry.addData("angle", angle);
                telemetry.addData("array v1", arrayV1);
                */

                telemetry.update();
            }
            
            //align the code to 50 milliseconds
            timeSleep(mainLoopStart);
            
            //increment the number of times that the loop has been run through
            ++loopCount1;
        }

    }

    /* things to add
    Return to Start                 Done
    Tune braking system curve?      NAH
    Heading control mode            DONE
    Gyro assisted strafing
    */


    //this is where all of the functions are defined.
    //that's about it.
    //oh yeah and they are just kinda thrown in there. There's no order

    //location tag
    //::FUNCTIONS

    //nobody uses this.
    //it's just calculating the angle with rollover.
    //it's still here here if you need this for some stuid reason
    private double getGlobalAngle()
    {
            //this is needed every time you want to actually talk to the imu
            //idk why and the documentation isn't helping
            Orientation angles = imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

            if (deltaAngle < -180)
                deltaAngle += 360;
            else if (deltaAngle > 180)
                deltaAngle -= 360;

            globalAngle += deltaAngle;
            lastAngles = angles;

            return globalAngle;
    }


    /*
    sets the power of the motors
    diagram of motors (top view):

      front
    v3 ----- v4
    |         |
    |         |
    |         |
    |         |
    v2 ----- v1

    the first line is the braking feature
    note that the right trigger is a float, not a bool
    the +0.1 is so that the robot just goes slow if it's all the way down, but
    doesn't completely stop
    */
    private void setPower()
    {
        addPower();
        bumperOffset = (1-gamepad1.right_trigger)+0.1;
        rightBack.setPower(motor[0]*bumperOffset);
        leftBack.setPower(motor[1]*bumperOffset);
        leftFrontal.setPower(motor[2]*bumperOffset);
        rightFrontal.setPower(motor[3]*bumperOffset);
        intake.setPower(motor[4]);
    }
    
    //copy of above function without logging, and with paramters
    //use the above function instead
    //this is only needed for the return to home (don't want to add logs while 
    //using them)
    private void setPower(double v1_in, double v2_in, double v3_in, double v4_in)
    {
        bumperOffset = (1-gamepad1.right_trigger)+0.1;
        rightBack.setPower(v1_in*bumperOffset);
        leftBack.setPower(v2_in*bumperOffset);
        leftFrontal.setPower(v3_in*bumperOffset);
        rightFrontal.setPower(v4_in*bumperOffset);
    }
   
    
    /*
    time in seconds. NOT NANOSECONDS
    returns the time in seconds since the start of the program.
    well actually since the reset of start
    but you know what I mean
    */
    private static long getTime()
    {
        return (System.nanoTime() - start) / 1000000;
    }


    /*
    this is the general way to asssign powers to a mechanum drivetrain

    breaking it down:
    r and robot angle combined effctively produce a point on the unit circle
    where the left joystick is, rotated pi/4 radians (45 degrees).

    so joystick straight up is in quadrant 1 at 45 degrees
    go grab a unit circle if you don't have it memorized, it'll be a useful aid

    rightX is just where the right stick is on its x axis

    don't worry about the headless code yet. I'll explain that in a minute

    I honestly don't know why this combination of cosines and sines gives the
    right powers. It just (sort of) does. See my rant in the motor defining
    section. So hey if you want to screw around and make work right PLEASE do.
    The + or - rightX is just to rotate the bot.
    Left side forward right side back, vice versa.

    So the headless part
    It basically adjusts the robot angle by a factor of actualAngle, which is
    where the robot is facing.
    the pi/180 is to convert the acutal angle to radians
    oh yeah this whole function is in radians
    the rest of the code should be in degrees
    if you want to screw with somebody change the += to a -=
    */
    private void generalPower()
    {
            r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
            rightX = gamepad1.right_stick_x;

            if (headless)
            {
                robotAngle += actualAngle*(Math.PI/180);
            }

            motor[0] = r * Math.sin(robotAngle - (Math.PI/4)) + rightX;
            motor[1] = r * Math.sin(robotAngle + (Math.PI/4)) - rightX;
            motor[2] = r * Math.sin(robotAngle - (Math.PI/4)) - rightX;
            motor[3] = r * Math.sin(robotAngle + (Math.PI/4)) + rightX;
            
            motor[4] = gamepad2.left_stick_x;
    }


    /*
    this sets the two angle variables to the correct values
    angle is cumulative (so if you turn to the right two rotations thats 720)
    ActualAngle is not (so if you do the above it's still 0)
    The offset is to change where the robot thinks is forward
    See next function
    The commented out stuff was me trying restrict the actualAngle + offset
    to the same range as the raw IMU output.
    I was unsucessful, but it would be nice if that was fixed at some point.
    */
    private void getAngle()
    {
            Orientation angles = imu.getAngularOrientation(
            AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            angle = getGlobalAngle();
            actualAngle = angles.firstAngle + offset;
    }

    //gets the offset
    //It just changes where the robot thinks is forward
    //sorry but idk how to explain it better
    private void offset()
    {
        Orientation angles = imu.getAngularOrientation(
            AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        offset = -angles.firstAngle;
    }

    //takes a true and returns a false
    //takes a false and returns a true
    //so yeah that's it
    
    //future alex note: wow am I an idiot
    //if you want to use this, on a variable input, try the following
    //input = !input
    //it does the same thing as this functon call
    public boolean switchInput(boolean input)
    {
        if (input == true){
            return false;
        } else {
            return true;
        }
    }

    /*
    rotates the robot to within 10 degrees of stright, as told by actualAngle
    the abs is just so that it can be 10 degrees off either way.
    pretty simple
    The light is blue if it's homing.
    Light blue for accurate (2 degree accuracy)
    Dark blue is for regular (10 degree accuracy)
    Actually I just added a quick check.
    If it's already within 10 degrees of striaight, it will go slowly with a max 
    error of 2 degrees.
    if the angle is less than 0, rotate so it's not
    if the angle is more than 0, rotate so it's not
    this is in a loop, so the opModeIsActive is needed here so as not to crash
    the robot (like I did. multiple times)
    because it also doesn't have the advantage of the main loop setting motor
    powers and updating the angle, both of those need to be done here
    telemetry is also done here, but significantly cut down from the main loop
    */
    private void returnToForward()
    {
        //define variables used for the timer
        long loopStartTime;
        
        //if within 10 degrees
        if (Math.abs(actualAngle) <= 10)
        {
            while (Math.abs(actualAngle) >= 2)
            {
                blinkLight.setConstant(color.lightBlue);
                opModeIsActive();
                loopStartTime = getTime();
                
                if (actualAngle <= 0)
                {
                    motor[0] = -0.1;
                    motor[1] = 0.1;
                    motor[2] = 0.1;
                    motor[3] = -0.1;
                    telemetry.addLine("if1");
                } else if (actualAngle >= 0) {
                    motor[0] = 0.1;
                    motor[1] = -0.1;
                    motor[2] = -0.1;
                    motor[3] = 0.1;
                    telemetry.addLine("if2");
                }
    
                setPower();
                getAngle();
                telemetry.addLine("Precisely Homing...");
                telemetry.addData("Actual Angle", actualAngle);
                telemetry.update();
                
                timeSleep(loopStartTime);
            }
            return;
        }
        
        
        //if not within 10 degrees at start
        while (Math.abs(actualAngle) >= 10)
        {
            blinkLight.setConstant(color.darkBlue);
            opModeIsActive();
            loopStartTime = getTime();
            
            if (actualAngle <= 0)
            {
                motor[0] = -1;
                motor[1] = 1;
                motor[2] = 1;
                motor[3] = -1;
                telemetry.addLine("if3");
            } else if (actualAngle >= 0) {
                motor[0] = 1;
                motor[1] = -1;
                motor[2] = -1;
                motor[3] = 1;
                telemetry.addLine("if4");
            }

            setPower();
            getAngle();
            telemetry.addLine("Homing...");
            telemetry.addData("Actual Angle", actualAngle);
            telemetry.update();
            
            timeSleep(loopStartTime);
        }
    }
    
    /*
    return home
    Just plays back the recorded values in reverse order. 
    Sets the light to green
    The asserts are just making sure that the arrays are all the same length
    They should be, but I would like the code to not shit itself
    Also stops on left bumper hit (same as the reset button)
    */
    void returnHome()
    {
        assert(arrayV1.size() == arrayV2.size());
        assert(arrayV3.size() == arrayV4.size());
        assert(arrayV1.size() == arrayV4.size());
        
        //define variables used for the return to home
        long loopStartTime = getTime();
        long loopLengthTime = 0;
        int elem;
        blinkLight.setConstant(color.green);
        
        
        while (arrayV1.size() != 0 && gamepad1.left_bumper != true){
            opModeIsActive();
            loopStartTime = getTime();
            
            elem = arrayV1.size()-1;
            
            setPower(arrayV1.get(elem),arrayV2.get(elem),arrayV3.get(elem),arrayV4.get(elem));
            
            arrayV1.remove(elem);
            arrayV2.remove(elem);
            arrayV3.remove(elem);
            arrayV4.remove(elem);
            
            telemetry.addLine("Returning to home...");
            telemetry.addData("Loop length (old)", loopLengthTime);
            telemetry.addData("Remaining Powers", arrayV1.size());
            telemetry.update();
            
            timeSleep(loopStartTime);
        }
        
    }
    
    //set home by clearing the motor power arrays
    void setNew()
    {
        arrayV1.clear();
        arrayV2.clear();
        arrayV3.clear();
        arrayV4.clear();
    }
    
    //add new point to the motor power arrays
    void addPower()
    {
        arrayV1.add(-motor[0]);
        arrayV2.add(-motor[1]);
        arrayV3.add(-motor[2]);
        arrayV4.add(-motor[3]);
    }
    
    //sets the robot to sleep so that each loop is 50 milliseconds long
    //crucial for return to home to work
    //to use just put a variable keeping track of the time when the loop started
    //pass that to this function
    void timeSleep (long startLoop)
    {
            long loopLength;
            loopLength = getTime()-startLoop;
            try
            {
                TimeUnit.MILLISECONDS.sleep(50 - loopLength);
            }
            catch(InterruptedException e)
            {
                
            }
            telemetry.addData("LoopLength", loopLength);
    }



}//put functions before this brace
//here we are at the end
