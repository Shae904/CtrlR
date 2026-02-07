package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name = "Axon Analog PIDF (CR, Same Power, Linear)", group = "Tuning")
public class CyclePIDFTuner extends LinearOpMode {

  // =========================
  // Internal state
  // =========================

  @Override
  public void runOpMode() {
    Robot robot = new Robot(this);
    ElapsedTime timer = new ElapsedTime();

    telemetry.addLine("Ready. Press START.");
    telemetry.update();

    waitForStart();
    timer.reset();

    while (opModeIsActive()) {

      if (gamepad1.aWasPressed()) {
        robot.cycleCW();

      }
      if (gamepad1.bWasPressed()) {
        robot.cycleCCW();
      }

      robot.spinSorter.updatePosition();
      double pidPower = robot.spinSorter.updatePIDControl();

      // ----- Telemetry -----
      telemetry.addData("targetIndex", "%d", robot.spinSorter.getTargetIndex());
      telemetry.addData("targetPos", "%.3f", robot.spinSorter.getTargetPos());
      telemetry.addData("pos", "%.3f", robot.spinSorter.getCurrentPos());
      telemetry.addData("error (signed)", "%.3f", robot.spinSorter.getError());
      telemetry.addData("lockedDir", "%d", robot.spinSorter.getLockedDir());
      telemetry.addData("approaching", "%b", robot.spinSorter.isApproaching());
      telemetry.addData("pid power", "%.4f", pidPower);
      telemetry.addData("PID", "P %.4f  F %.4f", SpinSorter.kP, SpinSorter.kF);
      telemetry.addData("output voltage", "%.3f", robot.servoPos.getVoltage());
      telemetry.addData("minV/maxV", "%.3f / %.3f", SpinSorter.minV, SpinSorter.maxV);
      if(robot.spinSorter.atTarget()){
          telemetry.addData("at target","true");
      }
      else{
          telemetry.addData("at target","false");
      }
      telemetry.update();
    }
  }
}
