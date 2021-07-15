package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp

public class lol_fgc_2018_go_brrr extends LinearOpMode {
    AnalogInput sensorfarleft,sensorfarright,sensornearright,sensornearleft;
    public DcMotorEx motorss,motords,motordf,motorsf;
    @Override
    public void runOpMode() throws InterruptedException {
        motordf = hardwareMap.get(DcMotorEx.class, "df");
        motords = hardwareMap.get(DcMotorEx.class, "ds");
        motorsf = hardwareMap.get(DcMotorEx.class, "sf");
        motorss = hardwareMap.get(DcMotorEx.class, "ss");
        sensorfarleft = hardwareMap.get(AnalogInput.class, "snfl");
        sensorfarright = hardwareMap.get(AnalogInput.class, "snfr");
        sensornearright = hardwareMap.get(AnalogInput.class, "snnr");
        sensornearleft = hardwareMap.get(AnalogInput.class, "snnl");

        motords.setDirection(DcMotorEx.Direction.REVERSE);
        motorss.setDirection(DcMotorEx.Direction.REVERSE);

        motordf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motords.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorsf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorss.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motordf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motords.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorsf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorss.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        waitForStart();
        while(!isStopRequested()){
            telemetry.addData("motordf: ", motordf.getCurrentPosition());
            telemetry.addData("motorsf: ", motorsf.getCurrentPosition());
            telemetry.addData("motords: ", motords.getCurrentPosition());
            telemetry.addData("motorss: ", motorss.getCurrentPosition());
            telemetry.addData("sensorfarleft: ", sensorfarleft.getVoltage());
            telemetry.addData("sensorfarright: ", sensorfarright.getVoltage());
            telemetry.addData("sensornearright: ", sensornearright.getVoltage());
            telemetry.addData("sensornearleft: ", sensornearleft.getVoltage());
            telemetry.update();
        }
    }
}
