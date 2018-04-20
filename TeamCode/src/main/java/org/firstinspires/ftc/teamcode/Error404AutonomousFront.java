package org.firstinspires.ftc.teamcode;

 /**
 *Error404AutonomousFront extends <code>Error404_Hardware_Tier2</code> and contains the state
  * machine that steps through each step in the front autonomous mission.
  *
  * This year's field presents a unique challenge for autonomous. The field is set up so that there
  * are four unique starting positions and your robot has to be able to start from any of them. We
  * could have four individual programs (one for each quadrant) but this is clumsy and opens us up
  * to bugs. Instead what we did, is we made two main autonomous classes that hold the framework code
  * for the front two qudrants and the rear two quadrants. Then we have four quadrant-specific child
  * classes that supply the quadrant-specific movement vaules.
 *
 * @author Team 8668
 * @see Error404_Hardware_Tier2
 */
public class Error404AutonomousFront extends Error404_Hardware_Tier2

{
    //////////////////////////////////////////////////////////////////////////////////
    /** This int variable sets the starting case value for the state machine (which step in the sequence to start with -- i.e. the first one or case 0) */
    private int state = 0;
    /** This int variable sets the starting encoder values for the motors. */
    private int encoder=0;
    /** This double initializes a timer that can be used during the autonomous. */
    private double timer=0;
    /** The distance from the balancing stone to the cryptobox. */
    protected int cryptoboxDriveDistance;
    /** The angle on the gyro needed to turn to face the cryptobox. */
    protected int turnToCryptobox;
    /** The angle on the gyro needed to turn away from the cryptobox to face the glyph pile. */
    protected int turnToPile;
    /** The distance needed to drive from the cryptobox to the glyph pile. */
    protected int driveToPile;
    /** The distance needed to drive from the glyph pile back to the cryptobox. */
    protected int backToCryptobox;
    /**
     * useExtendedGyro, if True, will be used in the gyro-controlled turn and drive methods to take
     * the navX +/- 180 degree gyro and read it like a 360 degree gyro. This lets the robot drive at
     * or turn to a heading of +/- 180 degrees without erroring out.
     *
     * If False, useExtendedGyro will not be used in the turn and drive methods and the hyro will be read normally.
     *  */
    protected boolean useExtendedGyro;
    /** A string that indicates which side of the field the robot is on ("blue" or "red") */
    private String fieldSide;
    /** A string that indicates side location on the field ("front" or "back"). */
    private String sideLocation;

    public Error404AutonomousFront()
    {
    }

    //////////////////////////////////////////////////////////////////////////////////
   /** Gets the gyro heading, sets the jewel servos to the home poition, and grabs the encoder
    * value of the leftFront drive motor for navigation purposes. */
    @Override public void init(){
        super.init();
        telemetry.addData("Gyro: ", getHeading());
        telemetry.addData("","V 1");
        arm.setPosition(0.1);
        swivel.setPosition(0.52);
        encoder=leftFront.getCurrentPosition();

        encoder=leftFront.getCurrentPosition();
    }


    //////////////////////////////////////////////////////////////////////////////////
    /** Determins the field position that robot is starting in and utilizes the corresponding
     * child class to get the move values for that quadrant of the field. */
    @Override public void start(){
        super.start();
    }

    public void setLocation(String frontOrBack, String redOrBlue){
        fieldSide = redOrBlue;
        sideLocation = frontOrBack;
    }

    protected boolean updateFromVuforia(String cryptoboxKey)
    {
        return true;
    }


    //////////////////////////////////////////////////////////////////////////////////
    /** Contains the state machine which contains the structure for each move the robot will take
     * during autonomous. In the cases where the robot will drive or turn, there are no
     * quadrant-specific values hard-coded in as these values will change depending on what quadrant
     * of the field the robot is starting in. Instead the Autonomous class sources the
     * quadrant-specific movement and turn values from field position-dependant child classes. */
    @Override public void loop ()
    {
        switch (state)
        {
            case 0: // Lower Arm
                arm.setPosition(0.79);
                    state++;
                break;

            case 1:  //Find Jewel
                timer =getRuntime();
                if(camera.getVoltage()<2.1){
                    telemetry.addData("On left","");
                    if(fieldSide.equals("BLUE")){
                        state=3;
                    }
                    if(fieldSide.equals("RED")){
                        state=2;
                    }
                    break;
                }
                else if(camera.getVoltage()>2.1){
                    telemetry.addData("On right","");
                    if(fieldSide.equals("BLUE")){
                        state=2;
                    }
                    if(fieldSide.equals("RED")){
                        state=3;
                    }
                    break;
                }

            case 2:  //Swivel Right
                if(((int)(getRuntime()-timer))>0.3) {
                    swivel.setPosition(0.7);
                    state=4;
                    timer =getRuntime();
                }
                break;

            case 3:  //Swivel Left
                if(((int)(getRuntime()-timer))>0.3) {
                    swivel.setPosition(0.4);
                    state=4;
                    timer =getRuntime();
                }
                break;

            case 4:  //Reset Arm
                if(((int)(getRuntime()-timer))>0.1) {
                    arm.setPosition(0.1);
                    swivel.setPosition(0.5);
                    state=6;
                    timer =getRuntime();
                }
                break;

            case 6:  //Read Pictograph
                if(updateFromVuforia(readCryptograph()))
                {
                    state= 7;
                }


                if(((int)(getRuntime()-timer))>2)
                {
                    setMultipleDirections("straight", "forward");
                    updateFromVuforia("CENTER");
                    state=7;
                }
                break;


            case 7:  //Drive to Cryptobox
                if(cryptoboxDriveDistance!=0 )
                {
                    if(cryptoboxDriveDistance>0)
                    {
                        driveStraightGyro(0.4, 40, 0, useExtendedGyro);
                    }
                    else
                    {
                        driveStraightGyro(-0.4, 40, 0, useExtendedGyro);
                    }

                    if (leftFront.getCurrentPosition() - encoder > Math.abs(cryptoboxDriveDistance))
                    {
                        //slide_sideways("RUE", 0, "l", 0);
                        stopEverything();
                        setMultipleDirections("turn", "right");
                        state++;
                        encoder = leftFront.getCurrentPosition();
                        timer =getRuntime();
                    }
                }
                else
                {
                    setMultipleDirections("turn", "right");
                    state++;
                    encoder = leftFront.getCurrentPosition();
                    timer =getRuntime();

                }

                break;

            case 8:  //Face Cryptobox
                if(turnToCryptobox!=0)
                {
                    if (pointTurnGyro(turnToCryptobox, useExtendedGyro) || getRuntime()-timer>5 )
                    {
                        state=10;
                        stopEverything();
                        setMultipleDirections("straight", "forward");
                        encoder = leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("straight", "forward");
                    state=10;
                    encoder = leftFront.getCurrentPosition();
                }
                break;


            case 10:  //Drive into Cryptobox
                driveStraightCombo(0.6);
                if(leftFront.getCurrentPosition()-encoder>93)
                {
                    stopEverything();
                    setMultipleDirections("straight", "reverse");
                    timer=getRuntime();
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 11:  //Deploy Glyph
                glyphIntake("out");
                state++;
                break;

            case 12:  //Wait
               // if(leftGlyph.getCurrentPosition()>20)
                if(getRuntime()-timer>1.2)
                {
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 13:  // Back Up
                driveStraightCombo(-0.8);
                if(leftFront.getCurrentPosition()-encoder>60)
                {
                    stopEverything();
                    setMultipleDirections("straight", "forward");
                    state++;
                    encoder=leftFront.getCurrentPosition();
                    timer = getRuntime();
                }
                break;

            case 14: // Push Glyph in
                glyphIntake("out");
                driveStraightCombo(0.5);
                if(getRuntime()-timer>1.1)
                {
                    driveStraight("RUE",0,"r",0);
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 15: //Back away from the cryptobox
                driveStraightCombo(-0.9);
                if(leftFront.getCurrentPosition()-encoder>60)
                {
                    stopEverything();
                    glyphIntake("stop");
                    if(fieldSide.equals("BLUE")){
                        setMultipleDirections("turn", "left");
                    }
                    else{
                        setMultipleDirections("turn", "right");
                    }
                    state ++;
                    encoder=leftFront.getCurrentPosition();
                    timer=getRuntime();
                }
                break;


            default:
                break;


        }
        telemetry.addData("1. State: ", state);
        telemetry.addData("2. Gyro: ", getHeading());
        telemetry.addData("3. Camera:  ", camera.getVoltage());
        telemetry.addData("4. Left Front Position: ", leftFront.getCurrentPosition());
        telemetry.addData("5. Delta Position: ", encoder);
        telemetry.addData("6. Cryptobox Drive Distance: ", cryptoboxDriveDistance);
        telemetry.addData("Pattern: ", readCryptograph());




    } // loop
} //
