package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public class GlyphIntake
{
    /** The leftGlyph motor spins the left glyph intake wheel. */
    protected DcMotor leftGlyph;
    /** The rightGlyph motor spins the right glyph intake wheel. */
    protected DcMotor rightGlyph;

    public void init(HardwareMap hardwareMap, Telemetry telemetry )
    {
        try {
            leftGlyph = hardwareMap.dcMotor.get("leftGlyph");
        } catch (Exception p_exeception) {
            telemetry.addData("leftGlyph not found in config file", 0);
            leftGlyph = null;
        }
        try {
            rightGlyph = hardwareMap.dcMotor.get("rightGlyph");
        } catch (Exception p_exeception) {
            telemetry.addData("rightGlyph not found in config file", 0);
            rightGlyph = null;
        }

    }

    public void in()
    {
        rightGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightGlyph.setDirection(REVERSE);
        leftGlyph.setDirection(REVERSE);
        leftGlyph.setPower(0.5);
        rightGlyph.setPower(0.5);
    }

    public void out()
    {
        rightGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightGlyph.setDirection(FORWARD);
        leftGlyph.setDirection(FORWARD);
        leftGlyph.setPower(0.5);
        rightGlyph.setPower(0.5);
    }

    public void outSlow()
    {
        rightGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightGlyph.setDirection(FORWARD);
        leftGlyph.setDirection(FORWARD);
        leftGlyph.setPower(0.2);
        rightGlyph.setPower(0.2);
    }

    public void stop()
    {
        rightGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftGlyph.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightGlyph.setDirection(FORWARD);
        leftGlyph.setDirection(REVERSE);
        leftGlyph.setPower(0);
        rightGlyph.setPower(0);
    }



}
