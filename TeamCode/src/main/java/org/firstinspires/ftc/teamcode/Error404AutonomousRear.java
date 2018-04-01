package org.firstinspires.ftc.teamcode;

 /**
 *Error404AutonomousRear extends <code>Error404_Hardware_Tier2</code> and contains the state
  * machine that steps through each step in the autonomous mission.
  *
  * This year's field presents a unique challenge for autonomous. The field is set up so that there
  * are four unique starting positions and your robot has to be able to start from any of them. We
  * could have four individual programs (one for each quadrant) but this is clumsy and opens us up
  * to bugs. Instead what we did is we made one main autonomous class that holds the framework code
  * that is the same for all four quadrants and then we have four quadrant-specific child classes
  * that supply the quadrant-specific movement vaules.
 *
 * @author Team 8668
 * @see Error404_Hardware_Tier2
 */
public class Error404AutonomousRear extends Error404_Hardware_Tier2

{
    //////////////////////////////////////////////////////////////////////////////////
    /** This int variable sets the starting case value for the state machine (which step in the sequence to start with -- i.e. the first one or case 0) */
    private int state = 0;
    private int encoder=0;
    private double timer=0;
    /** A move variable for driving from the glyph pile to the cryptobox. */
    protected int cryptoboxDriveDistance;
    /** A move variable for driving to line up on the glyph pile. */
    protected int glyphPileSlideDistance;
    /** A move variable for driving the robot into the glyph pile. */
    protected int driveIntoPile;
    /** Initializing the turning variable used for turning the correct side of the robot to face the cryptobox. */
    protected int turnToCryptobox;
    /** A move variable for turing the robot to face the glyph pile. */
    protected int turnToPile;
    /** Initializing the variable used for sliding the robot side to side to align on a particular column of the cryptobox. */
    protected int cryptoboxSlide;
    /** A string that indicates which side of the field the robot is on ("blue" or "red") */
    protected int slideAwayFromTheCryptobox;
    protected int driveToPile;
    protected int backToCryptobox;
    protected int slideBackToCryptobox;

    private String fieldSide;
    /** A string that indicates side location on the field ("front" or "back"). */
    private String sideLocation;

    public Error404AutonomousRear()
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
    /** Determins the field position that robot is starting in and utilizes the coorisponding
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
     * during autonomous. In the cases where the robot will drive somewhere, there are no values
     * hard-coded in as these values will change depending on what quadrant of the field the robot
     * is starting in. Instead the Autonomous class sources the quadrant-specific movement values
     * from field position-dependant child classes. */
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
                    setMultipleDirections("slide", "left");
                }


                if(((int)(getRuntime()-timer))>2)
                {
                    updateFromVuforia("CENTER");
                    setMultipleDirections("slide", "left");
                    state=7;
                }
                break;


            case 7:  // Slide to Glyph Pile
                if(glyphPileSlideDistance!=0)
                {
                    if(glyphPileSlideDistance>0)
                    {
                        slideSidewaysCombo(-0.7);
                    }
                    else
                    {
                        slideSidewaysCombo(0.7);
                    }

                    if(leftFront.getCurrentPosition()-encoder>Math.abs(glyphPileSlideDistance))
                    {
                        stopEverything();
                        setMultipleDirections("turn", "left");
                        state++;
                        encoder=leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("turn", "left");
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 8:  //Face Glyph Pile
                if(turnToPile!=0)
                {
                    if (pointTurnGyro(turnToPile))
                    {
                        state++;
                        stopEverything();
                        setMultipleDirections("straight", "forward");
                        encoder = leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("straight", "forward");
                    state++;
                    encoder = leftFront.getCurrentPosition();
                }
                break;

            case 9:  // Drive Into GLyph Pile
                if(driveIntoPile!=0)
                {
                    if(driveIntoPile>0)
                    {
                        driveStraightCombo(0.7);
                    }
                    else
                    {
                        driveStraightCombo(-0.7);
                    }

                    if(leftFront.getCurrentPosition()-encoder>Math.abs(driveIntoPile))
                    {
                        stopEverything();
                        setMultipleDirections("straight", "forward");
                        state++;
                        encoder=leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("straight", "forward");
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 10:  //Collect Glyphs From Pile
                glyphIntake("in");
                driveStraightCombo(0.3);
                if(leftFront.getCurrentPosition()-encoder>50)
                {
                    stopEverything();
                    setMultipleDirections("straight", "reverse");
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 11:  // Back up from glyph pile

                    driveStraightCombo(-0.7);
                    if(leftFront.getCurrentPosition()-encoder>50)
                    {
                        stopEverything();
                        if(fieldSide.equals("RED")){
                            setMultipleDirections("turn", "left");
                        }
                        else{
                            setMultipleDirections("turn", "right");
                        }
                        state++;
                        encoder=leftFront.getCurrentPosition();
                    }
                break;


            case 12:  //Orient on Cryptobox
                if(fieldSide.equals("RED")){
                    if (pointTurnGyro(turnToCryptobox))
                    {
                        state++;
                        stopEverything();
                        setMultipleDirections("straight", "reverse");
                        encoder = leftFront.getCurrentPosition();
                    }
                }
                else{
                    if (pointTurnGyro(turnToCryptobox))
                    {
                        state++;
                        stopEverything();
                        setMultipleDirections("straight", "reverse");
                        encoder = leftFront.getCurrentPosition();
                    }
                }
                break;

            case 13:  // Drive To Cryptobox

                        driveStraightCombo(-0.7);
                    if(leftFront.getCurrentPosition()-encoder>Math.abs(cryptoboxDriveDistance))
                    {
                        stopEverything();
                        if(fieldSide.equals("RED")){
                            setMultipleDirections("slide", "left");
                        }
                        else{
                            setMultipleDirections("slide", "right");
                        }
                        state++;
                        encoder=leftFront.getCurrentPosition();
                    }
                break;

            case 14:  // Slide to Cryptobox
                if(cryptoboxSlide!=0)
                {
                    if(cryptoboxSlide>0)
                    {
                        slideSidewaysCombo(0.7);
                    }
                    else
                    {
                        slideSidewaysCombo(-0.7);
                    }

                    if(leftFront.getCurrentPosition()-encoder>Math.abs(cryptoboxSlide))
                    {
                        stopEverything();
                        setMultipleDirections("straight", "reverse");
                        state++;
                        encoder=leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("straight", "reverse");
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 15:  //Drive into Cryptobox
                driveStraightCombo(-0.6);
                if(leftFront.getCurrentPosition()-encoder>40)
                {
                    stopEverything();
                    setMultipleDirections("straight", "forward");
                    timer=getRuntime();
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 16:  //Deploy Glyph
                glyphter.setPosition(0.22);
                state++;
                break;

            case 17:  //Wait
                if(getRuntime()-timer>2.5)
                {
                    glyphter.setPosition(.05);
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 18:  // Back Up
                driveStraightCombo(.7);
                if(leftFront.getCurrentPosition()-encoder>40)
                {
                    stopEverything();
                    setMultipleDirections("straight", "reverse");
                    state++;
                    encoder=leftFront.getCurrentPosition();
                    glyphIntake("stop");
                }
                break;
            case 19: // Push Glyph in
                driveStraightCombo(-0.3);
                if(leftFront.getCurrentPosition()-encoder>40)
                {
                    stopEverything();
                    setMultipleDirections("straight", "forward");
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;
            case 20:  //drive away from cryptobox into safe zone
                driveStraightCombo(.5);
                if(leftFront.getCurrentPosition()-encoder>100)
                {
                    stopEverything();
                    state++;
                    encoder=leftFront.getCurrentPosition();
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
        telemetry.addData("7. Cryptobox Slide: ", cryptoboxSlide);
        telemetry.addData("Pattern: ", readCryptograph());




    } // loop
} //
