package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.AnalogInput;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Tier 1 extends the <code>OpMode</code> class and captures all of our hardware and implements basic methods for
 * interacting with the motors.
 * Basic driving and robot operations are also achieved in this class.
 *
 * @author Team 8668
 * @see OpMode
 */
public class Error404_Hardware_Tier1 extends OpMode {
    /** This motor is one of the drive motors that makes up the robot drivetrain. */
    protected DcMotor leftFront;
    /** This motor is one of the drive motors that makes up the robot drivetrain. */
    protected DcMotor rightFront;
    /** This motor is one of the drive motors that makes up the robot drivetrain. */
    protected DcMotor leftRear;
    /** This motor is one of the drive motors that makes up the robot drivetrain. */
    protected DcMotor rightRear;
    /** The leftGlyph motor spins the left glyph intake wheel. */
    protected DcMotor leftGlyph;
    /** The rightGlyph motor spins the right glyph intake wheel. */
    protected DcMotor rightGlyph;

    /** The arm servo makes the jewel sword rotate down in preperation for knocking the correct jewel off. */
    protected Servo arm;
    /** The swivel servo rotates the jewel arm back and forth to knock off the correct jewel. */
    protected Servo swivel;
    /** The shoulder servo extends the relic arm over the field wall. */
    protected Servo shoulder;
    /** The elbow servo lifts the relic up and down. */
    protected Servo elbow;
    /** The hand servo opens and closes the relic claw. */
    protected Servo hand;
    /**  */
    protected Servo leftWhiskerServo;
    /**  */
    protected Servo rightWhiskerServo;

    /** The Rev Expansion Hub's own gryo and should only be used during initialization. */
    protected IntegratingGyroscope gyro;
    /** The navxMicro is a gyro and is used to record the robot's heading. */
    protected NavxMicroNavigationSensor navxMicro;
    /** The camera is used to differentiate colors during the jewel mission. */
    protected AnalogInput camera;
    /**  */
    protected AnalogInput leftWhisker;
    /**  */
    protected AnalogInput rightWhisker;


    /* The Vuforia system is used to track special patterns on the edge of the field. */

    /** Part of the vuforia system. */
    OpenGLMatrix lastLocation = null;
    /** The Vuforia system uses this. */
    VuforiaLocalizer vuforia;

    /** The Vuforia system uses this. */
    VuforiaTrackables relicTrackables;
    /** The Vuforia system uses this. */
    VuforiaTrackable relicTemplate;
    /** The Vuforia system uses this. */
    int cameraMonitorViewId;
    /** The Vuforia system uses this. */
    VuforiaLocalizer.Parameters parameters;
    /** The Vuforia system uses this. */
    RelicRecoveryVuMark vuMark;

    /**
     * When the driver selects an <code>OpMode</code> on the driver station,
     * this method initializes the hardware found in the <code>HardwareMap</code>.
     * If the device cannot be found in the config file, an error message
     * shows on the driver station telemetry.
     */
    @Override public void init() {
        /////////////////////////////////////////////////////////////////
        /* Attempting a hardware map of the motors, servos, and sensors//
        //      If the device cannot be found in the config file,      //
        //    an error message shows on the driver station telemetry.  //
        *////////////////////////////////////////////////////////////////
        try {
            arm = hardwareMap.get(Servo.class, "jewelSword");
        } catch (Exception p_exeception) {
            telemetry.addData("Jewel Sword not found in config file", 0);
            arm = null;
        }
        try {
            swivel = hardwareMap.get(Servo.class, "jewelSwivel");
            swivel.setPosition(0.52);
        } catch (Exception p_exeception) {
            telemetry.addData("Jewel Swivel not found in config file", 0);
            swivel = null;
        }
        try {
            shoulder = hardwareMap.get(Servo.class, "shoulder");
        } catch (Exception p_exeception) {
            telemetry.addData("shoulder servo not found in config file", 0);
            shoulder = null;
        }

        try {try {
            hand = hardwareMap.get(Servo.class, "hand");
        } catch (Exception p_exeception) {
            telemetry.addData("hand servo not found in config file", 0);
            hand = null;
        }
            elbow = hardwareMap.get(Servo.class, "elbow");
        } catch (Exception p_exeception) {
            telemetry.addData("elbow servo not found in config file", 0);
            elbow = null;
        }
        try {
            camera = hardwareMap.get(AnalogInput.class, "camera");
        } catch (Exception p_exeception) {
            telemetry.addData("camera not found in config file", 0);
            camera = null;
        }
        try {
            leftWhisker = hardwareMap.get(AnalogInput.class, "leftWhisker");
        } catch (Exception p_exeception) {
            telemetry.addData("left whisker not found in config file", 0);
            leftWhisker = null;
        }
        try {
            rightWhisker = hardwareMap.get(AnalogInput.class, "rightWhisker");
        } catch (Exception p_exeception) {
            telemetry.addData("right whisker not found in config file", 0);
            rightWhisker = null;
        }
        try {
            navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
            gyro = (IntegratingGyroscope)navxMicro;
        } catch (Exception p_exeception) {
            telemetry.addData("navx not found in config file", 0);
            navxMicro = null;
        }


        try {
            leftFront = hardwareMap.dcMotor.get("leftFront");
        } catch (Exception p_exeception) {
            telemetry.addData("leftFront not found in config file", 0);
            leftFront = null;
        }
        try {
            rightFront = hardwareMap.dcMotor.get("rightFront");
        } catch (Exception p_exeception) {
            telemetry.addData("rightFront not found in config file", 0);
            rightFront = null;
        }
        try {
            leftRear = hardwareMap.dcMotor.get("leftRear");
        } catch (Exception p_exeception) {
            telemetry.addData("leftRear not found in config file", 0);
            leftRear = null;
        }
        try {
            rightRear = hardwareMap.dcMotor.get("rightRear");
        } catch (Exception p_exeception) {
            telemetry.addData("rightRear not found in config file", 0);
            rightRear = null;
        }
        try {
            leftGlyph = hardwareMap.dcMotor.get("leftGlyph");
        } catch (Exception p_exeception) {
            telemetry.addData("leftGlyph not found in config file", 0);
            leftFront = null;
        }
        try {
            rightGlyph = hardwareMap.dcMotor.get("rightGlyph");
        } catch (Exception p_exeception) {
            telemetry.addData("rightGlyph not found in config file", 0);
            leftFront = null;
        }
        try {
            rightWhiskerServo = hardwareMap.get(Servo.class, "rightWhiskerServo");
        } catch (Exception p_exeception) {
            telemetry.addData("right whisker servo not found in config file", 0);
            rightWhiskerServo = null;
        }try {
            leftWhiskerServo = hardwareMap.get(Servo.class, "leftWhiskerServo");
        } catch (Exception p_exeception) {
            telemetry.addData("left whisker servo not found in config file", 0);
            leftWhiskerServo = null;
        }

        //RGB.setI2cAddress(I2cAddr.create8bit(0x3C));       //30 is the decimal conversion from 7 bit hexadecimal value 0x1e converted from 8 bit hexadecimal 0x3c
        //beacon.setI2cAddress(I2cAddr.create8bit(0x2C));
        //beacon2.setI2cAddress(I2cAddr.create8bit(0x5C));
        //RGB.enableLed(false); //not sure why these are needed here.  Seems to help reset the LEDS so the next enable commands are obeyed.
        //beacon.enableLed(false);
        //beacon2.enableLed(false);

    }

    /**
     * Detects cryptograph pattern.
     *
     * @return  a String denoting left, center, right, or blank if it can't determine the cryptograph.
     */
    public String readCryptograph(){
        String dejavu="";

        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            dejavu = vuMark.toString();
        }
        return dejavu;
    }
    public DcMotor convert(int mtr) {
        if (mtr == 1) {
            telemetry.addData("test",mtr);
            return leftFront;
        }
        if (mtr == 2) return rightFront;
        if (mtr == 3) return leftRear;
        if (mtr == 4) return rightRear;
        return null;
    }

    /** When the driver hits start sets up Vuforia. */
    public void start() {
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AXqihmr/////AAAAGfBzFvn3mUp2pArXwOs50RaJ5JQdpAr4rsjsH+U8jWqZz9IZH657T+p7j4SgiRhOxlbMsoXP43dcRWb953uxv1Pd9ykpvITS8R0LGB8w8DIEYElzCWAvx0qxFO/6mUq2nuWvAhSyGbVsQk3IgjC17DwijqO1i21E7bZtAp3LRfUaNjvwh38Q0EZkIY0ulaUChjb/sep2XzJ8/yoOxq3deuAVx6pSPcQwaLpdV7vSvLr7rDr1OIOZeb5DGjAEA4QLiV/t8/daIVi3AAWTpCi0kskgtT/KZMzzok8ACYE96pDMKn7Z5epuguKyZ4/6w9Mc7oMF68XMbtf60AhZvgUApJCakYrDT9MwT7IpGa03e+HC";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
    }
    public void loop() { }
    public void stop() { }
    ////////////////////////////////////////////
    /*   Methods that return raw data use     //
    //       for decision making and          //
    //  and String versions for printing to   //
    //        Driver's Station phone          //
    *///////////////////////////////////////////
    public double get_power(DcMotor motor)
    {
        double motorReturn = 0;
        if(motor != null)
        {
            motorReturn = motor.getPower();
            return motorReturn;
        }
        return motorReturn;
    }

    /**
     * Gets the motor's position and power
     *
     * @param motor  the motor whose position and power will be recorded
     * @return  a String which is the motor's power
     */
    public String get_power_tele(DcMotor motor)
    {
        String motorReturn = "";
        if(motor != null){
            motorReturn += motor.getPower();
            return motorReturn;
        }
        motorReturn += "NULL";
        return motorReturn;
    }

    /**
     * Get the motor's position
     *
     * @param motor  the motor whose position will be recorded
     * @return  the motor's posisiton
     */
    public int get_position(DcMotor motor)
    {
        int motorReturn = 0;
        if (motor != null)
        {
            motorReturn = motor.getCurrentPosition();
            return motorReturn;
        }
        return motorReturn;
    }

    /**
     * Get the motor's current encoder count
     *
     * @param motor  the motor whose encoder counts will be recorded
     * @return  the motor's encoder count
     */
    public String get_position_tele(DcMotor motor)
    {
        String motorReturn = "";
        if(motor != null)
        {
            motorReturn += motor.getCurrentPosition();
            return motorReturn;
        }
        motorReturn += "NULL";
        return motorReturn;
    }

    /**
     * Get the motor's current mode
     *
     * @param motor  the motor whose mode will be checked
     * @return  the motor's mode
     */
    public String get_mode(DcMotor motor) {
        String motorReturn = "";
        if (motor != null) {
            motorReturn += motor.getMode();
            return motorReturn;
        }
        motorReturn += "NULL";
        return motorReturn;
    }

    /**
     * Get the motor's current direction
     *
     * @param motor  the motor whose direction will be recorded
     * @return  the motor's direction
     */
    public String get_direction(DcMotor motor) {
        String motorReturn = "";
        if (motor != null) {
            motorReturn += motor.getDirection();
            return motorReturn;
        }
        motorReturn += "NULL";
        return motorReturn;
    }

    //////////////////////////////////////////
    /*method that checks if the motor has   //
    // reached it's goal if found, else     //
    //          returns false.              //
    */////////////////////////////////////////

    public boolean is_encoder_reached(int goal, DcMotor motor)
    {
        int encoderCount = get_position(motor);
        // encoderCount=Math.abs(encoderCount);
        if(encoderCount >= goal)
        {return true;}
        else if((encoderCount > (goal - 10)) && (encoderCount < (goal + 10)))
        {return true;}
        else
        {return false;}
    }

    //////////////////////////////////////////////
    /*  method that checks if motors are reset.  //
    // If found returns true, else returns false //
    */////////////////////////////////////////////
    public boolean is_encoder_reset(DcMotor motor)
    {
        if(get_position(motor) == 0)
        {return true;}
        else
        {return false;}
    }

    ///////////////////////////////////////////
    /* methods that resets encoders if found,//
    //          else does nothing.           //
    *//////////////////////////////////////////
   /* public void reset_encoder(DcMotor motor)
    {
        if(motor != null)
        {
            motor.setMode(RESET_ENCODERS);
        }
    }
*/

    /**
     * Used to get the robot's heading.
     *
     * @return  the robtot's heading as an Int
     */
    public int getHeading(){
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            return (int)AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle));
        }



    ////////////////////////////////////////
    // In these set power methods, the    //
    //method checks to see if the motor   //
    //is null. If so, it skips that motor.//
    //If it is not null, the power is set //
    //to that motor.                      //
    ////////////////////////////////////////

    /**
     * If the motor is valid, set the motor to a certain power
     *
     * @param power  a double the is the power to be set to the motor
     * @param motor  The motor whose power will be set.
     */
    public void set_power(double power, DcMotor motor){
        if (motor != null) {
            motor.setPower(power);
        }
    }

    ///////////////////////////////////////////////////////
    // This set mode method uses two parameters:         //
    // motor and a 3-4 letter mode abbreviation.         //
    // If the motor is not null, the mode will be set to://
    //RTP= Run to Position       //////////////////////////
    //RUE= Run using encoders    //
    //RWOE= Run without encoders //
    ///////////////////////////////

    /**
     * If the motor is valid, set it with 3 different available modes (RUN_TO_POSITION,
     * RUN-USING_ENCODERS, RUN_WITHOUT_ENCODERS).
     *
     * @param motor  The motor whose modes will be set.
     * @param modetoset  The different modes to be set
     */
    public void set_mode(DcMotor motor, String modetoset){
        modetoset=modetoset.toUpperCase();
        if (motor != null){
            if (modetoset.equals("RTP")){
                motor.setMode(RUN_TO_POSITION);
            }
            if (modetoset.equals("RUE")){
                motor.setMode(RUN_USING_ENCODER);
            }
            if (modetoset.equals("RWOE")){
                motor.setMode(RUN_WITHOUT_ENCODER);
            }
        }
    }


    ///////////////////////////////////////////////////////////
    //This set direction method takes two parameters: Motor  //
    // and direction. The direction is set as F for forward  //
    // and R for reversed. If the motor is not null, the     //
    //direction is set.                                      //
    ///////////////////////////////////////////////////////////

    /**
     * If the motor is valid, set the motor direction as either forward (f) or reverse (r).
     *
     * @param motor  The motor whose direction will be set
     * @param direction  a String that sets the motor's direction.
     */
    public void set_direction(DcMotor motor, String direction) {
        if (motor != null) {
            direction=direction.toLowerCase();
            if (direction.equals("r")) {
                motor.setDirection(DcMotor.Direction.REVERSE);
            }
            if (direction.equals("f")) {
                motor.setDirection(DcMotor.Direction.FORWARD);
            }
        }
    }

    //////////////////////////////////////////
    //This method takes two parameters, one //
    //for the motor and one for the desired //
    //position. It then sets the position   //
    //to the motor if the motor is not null.//
    //////////////////////////////////////////

    /**
     * Tells the motor to go to a certain position.
     *
     * @param motor  The motor whose position will be set.
     * @param position  the desired posistion for the motor to achieve.
     */
    public void set_position(DcMotor motor, int position)
    {
        if (motor != null){
            motor.setTargetPosition(position);
        }
    }

    //////////////////////////////////////////////////
    //In this method, you input the desired distance//
    //in inches, the wheel diameter, and the gear   //
    //ratio. This method will then calculate the    //
    //needed number of encoder ticks needed to      //
    //drive the distance input.                     //
    //////////////////////////////////////////////////

    /**
     * converts inches to encoder counts
     *
     * @param desiredDistance  an Int that is the target distance
     * @param wheel_diameter  a Double that is your wheel diameter
     * @param gear_ratio  a Double that is the gear ratio
     * @return  the needed number of encoder ticks to move the target amount of inches
     */
    public int distance2encoder(int desiredDistance, double wheel_diameter, double gear_ratio) {
        return (int) ( 280*(desiredDistance/(((3.14159265)*(wheel_diameter))*gear_ratio)));}

    ///////////////////////////////////
    //This scale motor power method  //
    //takes the raw power input from //
    //joysticks and converts it to   //
    //floats. With these set values, //
    //the motor power will ramp up   //
    //instead of being sudden & jerky//
    //////////////////////////////// //
    public float scale_motor_power (float p_power)
    {
        float l_scale = 0.0f;
        float l_power = Range.clip(p_power, -1, 1);
        float[] l_array =
                { 0.00f, 0.05f, 0.09f, 0.10f, 0.12f
                        , 0.15f, 0.18f, 0.24f, 0.30f, 0.36f
                        , 0.43f, 0.50f, 0.60f, 0.72f, 0.85f
                        , 1.00f, 1.00f
                };
        int l_index = (int)(l_power * 16.0);
        if (l_index < 0)
        {
            l_index = -l_index;
        }
        else if (l_index > 16)
        {
            l_index = 16;
        }

        if (l_power < 0)
        {
            l_scale = -l_array[l_index];
        }
        else
        {
            l_scale = l_array[l_index];
        }

        return l_scale;

    }
}
