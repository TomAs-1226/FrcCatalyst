package frc.lib.catalyst.util;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

/**
 * Ported with the gate from team 5805's {@code VisionShotGateTest}; the two cases that exercised the
 * robot's own manual-steering reacquisition stay in that repository.
 */
class VisionShotGateTest {
  private VisionShotGate cameraQualified(double speed) {
    var gate=new VisionShotGate();
    for(int i=0;i<=50;i++) {
      double now=1+i*.01;
      gate.recordMotion(now,speed,0);
      if(i>=10 && i%5==0) {
        double capture=now-.08;
        gate.recordCamera(capture,9+speed*(capture-1),4,.02,.06);
      }
    }
    assertTrue(gate.canScore(1.5,0,speed,0,.9));
    return gate;
  }

  /** A parked robot {@code hubDistance} from the hub, one camera frame every 50 ms with alternating scatter. */
  private VisionShotGate parkedAt(double hubDistance,double sigma,double scatter) {
    var gate=new VisionShotGate();
    gate.setTarget(11.9,4.0);
    for(int i=0;i<=50;i++) {
      double now=1+i*.01;
      gate.recordMotion(now,0,0);
      if(i>=10 && i%5==0) gate.recordCamera(now-.08,11.9-hubDistance+(i%10==0?scatter:-scatter),4.0,.02,sigma);
    }
    return gate;
  }

  @Test void farShotsAreHeldToTheSameAimErrorAsCloseOnesNotToAFixedTenCentimetres() {
    // CAMIS Q63: one hub tag past 2.6 m is modelled worse than 10 cm, and every shot past 3.2 m was refused.
    assertTrue(parkedAt(4.0,.15,.06).canScore(1.5,0,0,0,.9), "a 4 m single-tag shot qualifies");
    assertTrue(parkedAt(4.8,.19,.08).canScore(1.5,0,0,0,.9), "and a 4.8 m one");
    // Close in, nothing loosens: 10 cm is already 2 degrees at 2.9 m.
    assertFalse(parkedAt(2.0,.15,.0).canScore(1.5,0,0,0,.9), "a weak frame at 2 m still cannot qualify");
    assertFalse(parkedAt(2.0,.06,.06).canScore(1.5,0,0,0,.9), "12 cm of scatter at 2 m is still a mismatch");
    assertFalse(parkedAt(4.0,.06,.09).canScore(1.5,0,0,0,.9), "18 cm of scatter at 4 m is still a mismatch");
    assertEquals(.10,new VisionShotGate().agreementMeters(Double.NaN),0,"without a hub every limit stays at its floor");
    assertEquals(.20,new VisionShotGate().agreementMeters(9),0,"and never past 20 cm");
  }

  @Test void movingAndParkedShotsContinueThroughBriefCameraLossWithoutRefreshingTheCaptureClock() {
    for(double speed:new double[]{0,.6}) {
      var gate=cameraQualified(speed);
      double capture=gate.captureSeconds();
      for(int i=51;i<=140;i++) {
        double now=1+i*.01;
        gate.recordMotion(now,speed,0);
        assertTrue(gate.canAim(now,0,speed,.9),gate.captureStatus());
        assertTrue(gate.canScore(now,0,speed,0,.9),gate.scoreBlocker(now,0,speed,0,.9));
        assertEquals(capture,gate.captureSeconds(),0,"wheel updates never manufacture a camera fix");
      }
      assertTrue(gate.usingCarry(2.4));
    }
  }

  @Test void continuousNativeAndDuplicateCameraUpdatesCannotExtendTheTimeBudget() {
    var gate=cameraQualified(0);
    for(int i=51;i<=200;i++) {
      double now=1+i*.01;
      gate.recordMotion(now,0,0);
      gate.recordCamera(1.42,9,4,.02,.06); // Repeated old capture, not new evidence.
    }
    assertEquals(1.58,gate.carryAgeSeconds(3),1e-9);
    assertFalse(gate.canScore(3,0,0,0,.9));
    assertTrue(gate.scoreBlocker(3,0,0,0,.9).contains("time budget"));
  }

  @Test void returningToTheSameSpotCannotEraseTheCarryTravelBudget() {
    var gate=cameraQualified(.75);
    for(int i=51;i<=180;i++) gate.recordMotion(1+i*.01,i<110?.75:-.75,0);
    assertTrue(gate.carryAgeSeconds(2.8)<new VisionShotGate.Config().carrySeconds());
    assertTrue(gate.carryTravelMeters()>new VisionShotGate.Config().carryMeters());
    assertFalse(gate.canScore(2.8,0,.75,0,.9));
    assertTrue(gate.scoreBlocker(2.8,0,.75,0,.9).contains("travel budget"));
  }

  @Test void staleUnknownGappedAndBackwardsNativeMotionRevokeCarry() {
    var gate=cameraQualified(.6);
    assertFalse(gate.canScore(1.7,0,.6,0,.9),"a claimed fresh native age cannot replace missing history");
    for(double invalid:new double[]{Double.NaN,Double.POSITIVE_INFINITY,-.1}) {
      gate=cameraQualified(.6);
      gate.recordMotion(1.51,.6,0);
      assertFalse(gate.canScore(1.51,invalid,.6,0,.9));
    }
    gate=cameraQualified(.6);gate.recordMotion(1.51,Double.NaN,0);
    assertFalse(gate.canScore(1.51,0,.6,0,.9));
    gate=cameraQualified(.6);gate.recordMotion(1.60,.6,0);
    assertFalse(gate.canScore(1.60,0,.6,0,.9));
    gate=cameraQualified(.6);gate.recordMotion(1.49,.6,0);
    assertFalse(gate.canScore(1.51,0,.6,0,.9));
  }

  @Test void aNewFaultRejectsOldAndDelayedCameraEvidenceUntilANewTrajectoryIsQualified() {
    var gate=cameraQualified(.6);
    gate.invalidateBefore(1.6);
    for(int i=51;i<=75;i++) gate.recordMotion(1+i*.01,.6,0);
    gate.recordCamera(1.59,9+.6*.59,4,.02,.06);
    assertFalse(gate.canScore(1.75,0,.6,0,.9));
    for(int i=76;i<=105;i++) {
      double now=1+i*.01;
      gate.recordMotion(now,.6,0);
      if(i%5==0) {
        double capture=now-.08;
        gate.recordCamera(capture,9+.6*(capture-1),4,.02,.06);
      }
    }
    assertTrue(gate.canScore(2.05,0,.6,0,1.6));
  }

  @Test void aGoodCameraReturnDoesNotInterruptTheExistingCarryWhileANewWindowBuilds() {
    var gate=cameraQualified(.6);
    for(int i=51;i<=80;i++) gate.recordMotion(1+i*.01,.6,0);
    gate.recordCamera(1.72,9+.6*.72,4,.02,.06);
    assertTrue(gate.canScore(1.8,0,.6,0,.9));
    assertEquals(.38,gate.carryAgeSeconds(1.8),1e-9,"one return frame cannot renew the qualified anchor");
  }

  @Test void aContradictoryOrPoorQualityReturnRevokesTheCarryImmediately() {
    var gate=cameraQualified(.6);
    for(int i=51;i<=80;i++) gate.recordMotion(1+i*.01,.6,0);
    gate.recordCamera(1.72,9+.6*.72+.14,4,.02,.06);
    assertFalse(gate.canScore(1.8,0,.6,0,.9));
    gate=cameraQualified(.6);
    gate.recordCamera(1.45,9+.6*.45,4,.16,.06);
    assertFalse(gate.canScore(1.5,0,.6,0,.9));
  }

  @Test void accumulatingCameraWheelContradictionCannotBeMaskedByCarryOrReturnAfterTagLoss() {
    var gate=new VisionShotGate();
    boolean wasQualified=false,sawMismatch=false;
    for(int i=0;i<=55;i++) {
      double now=1+i*.01;
      gate.recordMotion(now,0,0);
      if(i>=10 && i%5==0) {
        double capture=now-.08;
        gate.recordCamera(capture,14+.3*(capture-1),4,.02,.06);
        sawMismatch|=gate.captureStatus().contains("mismatch");
        wasQualified|=gate.canScore(now,0,0,0,.9);
      }
    }
    assertTrue(wasQualified,"the slow drift must first earn the former carry token");
    assertTrue(sawMismatch,"the complete camera window must expose the accumulating contradiction");
    assertFalse(gate.canScore(1.55,0,0,0,.9));
    for(int i=56;i<=75;i++) {
      double now=1+i*.01;
      gate.recordMotion(now,0,0);
      assertFalse(gate.canScore(now,0,0,0,.9),"dropping tags cannot resurrect rejected evidence");
    }
    for(int i=76;i<=110;i++) {
      double now=1+i*.01;
      gate.recordMotion(now,0,0);
      if(i%5==0) gate.recordCamera(now-.08,14+.3*.47,4,.02,.06);
    }
    assertTrue(gate.canScore(2.1,0,0,0,.9),"a genuinely clean new trajectory may recover");
  }

  @Test void disturbedModelRevokesCarryWithoutErasingQualifiedFreshCameraShots() {
    var gate=cameraQualified(.6);
    for(int i=51;i<=80;i++) {
      double now=1+i*.01;
      gate.recordMotion(now,.6,0);
      // 33 Hz / 80 ms latency keeps capture age below the unchanged 120 ms fresh-only limit.
      if(i<=65 && i%3==0) {
        double capture=now-.08;
        gate.recordCamera(capture,9+.6*(capture-1),4,.02,.06);
      }
      gate.invalidateCarryBefore(now);
      if(i<=65) assertTrue(gate.canScore(now,0,.6,0,.9),"measured camera motion remains usable");
    }
    assertFalse(gate.canScore(1.8,0,.6,0,.9),"disturbed motion cannot borrow the blind carry");
  }

  private VisionShotGate moving(double vx,double vy,double ax,double ay) {
    var gate=new VisionShotGate();
    for(int i=0;i<=6;i++) {
      double t=i*.05;
      gate.record(1+t,14+vx*t+.5*ax*t*t,4+vy*t+.5*ay*t*t,.03,.07,
          vx+ax*t,vy+ay*t);
    }
    return gate;
  }
  @Test void bothParkedAndMovingShotsUseRealCameraAndMeasuredVelocity() {
    assertTrue(moving(0,0,0,0).canScore(1.32,.01,0,0,.9));
    assertTrue(moving(1,.4,0,0).canScore(1.32,.01,1.08,.3,.9));
    assertTrue(moving(.2,.1,2,-.5).canScore(1.32,.01,.8,.4,.9));
  }
  @Test void stationaryCameraCannotCertifySpinningWheelsOrWrongMotionDirection() {
    var gate=new VisionShotGate();
    for(int i=0;i<=6;i++) gate.record(1+i*.05,14,4,.03,.07,1,0);
    assertFalse(gate.canScore(1.32,.01,1,0,.9));
    gate=new VisionShotGate();
    for(int i=0;i<=6;i++) gate.record(1+i*.05,14+i*.05,4,.03,.07,-1,0);
    assertFalse(gate.canScore(1.32,.01,1,0,.9));
  }
  @Test void cameraLossFutureFixAndStaleNativeFrameCannotCarryThisMode() {
    var gate=moving(1,0,0,0);
    assertFalse(gate.canScore(1.43,.01,1,0,.9));
    assertFalse(gate.canScore(1.29,.01,1,0,.9));
    assertFalse(gate.canScore(1.32,.036,1,0,.9));
    assertFalse(gate.canScore(1.32,-.01,1,0,.9));
    assertFalse(gate.canScore(1.32,.01,Double.NaN,0,.9));
    assertFalse(gate.canScore(1.32,.01,1,Double.NaN,.9));
  }
  @Test void preImpactHistoryCannotSatisfyPostImpactChecks() {
    var gate=moving(0,0,0,0);
    assertFalse(gate.canScore(1.32,.01,0,0,1.15));
    gate.record(1.35,14,4,.01,.05,0,0);gate.record(1.4,14,4,.01,.05,0,0);
    assertTrue(gate.canScore(1.42,.01,0,0,1.15));
  }
  @Test void weakerPrecisionPreservesPriorEvidenceButInvalidOrDisagreeingFramesRevokeIt() {
    var gate=moving(0,0,0,0);gate.record(1.31,14,4,.16,.05,0,0);
    assertFalse(gate.canAim(1.32,.01,0,.9));
    gate=moving(0,0,0,0);gate.record(1.31,14,4,.01,.101,0,0);
    assertTrue(gate.canScore(1.32,.01,0,0,.9));
    assertEquals(1.3,gate.captureSeconds(),1e-9,"weaker precision cannot refresh the accepted capture");
    gate=moving(0,0,0,0);gate.record(1.31,Double.NaN,4,.01,.05,0,0);
    assertFalse(gate.canAim(1.32,.01,0,.9));
  }

  @Test void losingOneOfTwoTagsAtThreeMetersKeepsOnlyTheExistingBoundedCarry() {
    var gate=cameraQualified(.6);
    double capture=gate.captureSeconds();
    double singleTagSigma=.05*(1+.15*3*3);
    assertEquals(.05875,singleTagSigma/2,1e-9);
    assertEquals(.1175,singleTagSigma,1e-9);
    for(int i=51;i<=200;i++) {
      double now=1+i*.01;
      gate.recordMotion(now,.6,0);
      double weakCapture=now-.08;
      gate.recordCamera(weakCapture,9+.6*(weakCapture-1),4,.02,singleTagSigma);
      assertEquals(capture,gate.captureSeconds(),0,"remaining weak tag never refreshes capture time");
      if(now<=capture+new VisionShotGate.Config().carrySeconds())
        assertTrue(gate.canScore(now,0,.6,0,.9),"tag-count handoff keeps the existing qualified carry");
    }
    assertFalse(gate.canScore(3,0,.6,0,.9),"continuous weaker frames cannot extend the time budget");
  }

  @Test void contradictoryWeakFramesAndFaultsStillRevokePriorCarry() {
    var gate=cameraQualified(.6);
    gate.recordMotion(1.51,.6,0);
    gate.recordCamera(1.43,9+.6*.43+.14,4,.02,.1175);
    assertFalse(gate.canScore(1.51,0,.6,0,.9));
    assertEquals("weaker camera contradicts measured motion",gate.captureStatus());
    gate=cameraQualified(.6);
    gate.recordCamera(gate.captureSeconds(),9+.6*.42+.11,4,.02,.1175);
    assertFalse(gate.canScore(1.5,0,.6,0,.9),"equal-time disagreement is checked before sigma");
    gate=cameraQualified(.6);gate.invalidateBefore(1.51);
    gate.recordCamera(1.52,9+.6*.52,4,.02,.1175);
    assertFalse(gate.canAim(1.52,0,.6,.9));
    gate=cameraQualified(.6);gate.recordMotion(1.51,Double.NaN,0);
    gate.recordCamera(1.43,9+.6*.43,4,.02,.1175);
    assertFalse(gate.canAim(1.51,0,.6,.9));
  }

  @Test void allPeerOrdersDetectWeakFirstAndNonFirstPairConflictsWithoutCountingPeersAsFixes() {
    var orders=java.util.List.of(new int[]{0,1,2},new int[]{0,2,1},new int[]{1,0,2},
        new int[]{1,2,0},new int[]{2,0,1},new int[]{2,1,0});
    for(int[] order:orders) {
      var conflict=cameraQualified(0);
      double[] x={9,9.09,8.91}; double[] sigma={.1175,.06,.06};
      for(int i:order) conflict.recordCamera(1.44,x[i],4,.02,sigma[i]);
      assertFalse(conflict.canScore(1.5,0,0,0,.9),"pairwise conflict must not depend on first camera");
      assertEquals("simultaneous camera disagreement",conflict.captureStatus());
      var agreeing=cameraQualified(0);
      double[] close={9,9.01,8.99};
      for(int i:order) agreeing.recordCamera(1.44,close[i],4,.02,sigma[i]);
      assertTrue(agreeing.canScore(1.5,0,0,0,.9));
      assertEquals(1.44,agreeing.captureSeconds(),0,"agreeing peer order supplies only one capture time");
    }
  }
  @Test void captureGapsAndDuplicateFramesCannotManufactureAMotionWindow() {
    var gate=new VisionShotGate();
    for(int i=0;i<20;i++) gate.record(1,14,4,.01,.05,0,0);
    assertFalse(gate.canScore(1.02,.01,0,0,.9));
    gate=moving(0,0,0,0);gate.record(1.5,14,4,.01,.05,0,0);
    assertFalse(gate.canScore(1.52,.01,0,0,.9));
  }

  @Test void equalCaptureConflictingCameraRevokesAlreadyQualifiedShotAndCarry() {
    var gate=moving(0,0,0,0);
    assertTrue(gate.canScore(1.32,.01,0,0,.9));
    double capture=gate.captureSeconds();
    gate.record(capture,14.11,4,.01,.05,0,0);
    assertFalse(gate.canAim(1.32,.01,0,.9));
    assertEquals("simultaneous camera disagreement",gate.captureStatus());
    gate.record(capture,14,4,.01,.05,0,0);
    assertFalse(gate.canScore(1.32,.01,0,0,.9),"a replay cannot undo the conflict epoch");
  }

  @Test void pendingEqualCaptureConflictCannotBecomeEvidenceWhenMotionArrives() {
    var gate=new VisionShotGate();
    gate.recordMotion(1,0,0);
    gate.recordCamera(1.02,14,4,.01,.05);
    gate.recordCamera(1.02,14.11,4,.01,.05);
    gate.recordMotion(1.02,0,0);
    assertTrue(Double.isNaN(gate.captureSeconds()));
    assertEquals("simultaneous camera disagreement",gate.captureStatus());
  }

  @Test void agreeingEqualCapturePeersDoNotManufactureElapsedCameraMotion() {
    var gate=new VisionShotGate();
    gate.recordMotion(1,0,0);
    gate.recordCamera(1,14,4,.01,.05);
    gate.recordCamera(1,14.01,4,.01,.05);
    gate.recordCamera(1,13.99,4,.01,.05);
    assertEquals(1,gate.captureSeconds(),0);
    assertFalse(gate.canScore(1.02,.01,0,0,.9));
  }
  @Test void delayedCameraFramesUseCaptureTimeVelocityDuringFastDirectionChanges() {
    var aligned=new VisionShotGate();
    var oldPairing=new VisionShotGate();
    int alignedBlocks=0,oldBlocks=0;
    double latency=.10;
    for(int i=0;i<=250;i++) {
      double now=i*.02;
      double currentV=.74*Math.sin(7*now);
      aligned.recordMotion(now,currentV,0);
      if(i%2!=0 || now<latency) continue;
      double capture=now-latency;
      double x=14+(.74/7)*(1-Math.cos(7*capture));
      aligned.recordCamera(capture,x,4,.03,.07);
      // Reproduce the previous production bug: current wheel velocity on an older image.
      oldPairing.record(capture,x,4,.03,.07,currentV,0);
      if(now>.8) {
        if(!aligned.canScore(now,.01,Math.abs(currentV),0,0)) alignedBlocks++;
        if(!oldPairing.canScore(now,.01,Math.abs(currentV),0,0)) oldBlocks++;
      }
    }
    assertTrue(oldBlocks>=3,"counterexample must reproduce moving-shot interruptions");
    assertEquals(0,alignedBlocks,"capture-aligned motion remains valid through repeated reversals");
  }
  @Test void fractionalCapturesRequireInterpolatedVelocityNotEitherEndpoint() {
    var aligned=new VisionShotGate();
    var lowerEndpoint=new VisionShotGate();
    var upperEndpoint=new VisionShotGate();
    // Analytic constant acceleration isolates interpolation from camera/plant noise. The high
    // acceleration amplifies a half-frame error; this helper test does not claim a safe shot speed.
    double acceleration=20;
    for(int i=0;i<=16;i++) {
      double elapsed=i*.04,now=1+elapsed;
      aligned.recordMotion(now,acceleration*elapsed,0);
      if(elapsed<.10) continue;
      double captureElapsed=elapsed-.10,capture=1+captureElapsed;
      double x=5+.5*acceleration*captureElapsed*captureElapsed;
      aligned.recordCamera(capture,x,4,.02,.06);
      lowerEndpoint.record(capture,x,4,.02,.06,acceleration*(captureElapsed-.02),0);
      upperEndpoint.record(capture,x,4,.02,.06,acceleration*(captureElapsed+.02),0);
    }
    assertTrue(aligned.canScore(1.64,.01,12.8,0,.9),aligned.captureStatus());
    assertFalse(lowerEndpoint.canScore(1.64,.01,12.8,0,.9),
        "borrowing the lower bracket endpoint must fail this counterexample");
    assertFalse(upperEndpoint.canScore(1.64,.01,12.8,0,.9),
        "borrowing the upper bracket endpoint must fail this counterexample");
  }
  @Test void captureWaitsForABracketRatherThanBorrowingTheNewestVelocity() {
    var gate=new VisionShotGate();
    gate.recordMotion(1,0,0);
    gate.recordCamera(1.01,14,4,.01,.05);
    assertFalse(gate.canAim(1.02,.01,0,.9));
    assertTrue(gate.captureStatus().contains("waiting"));
    gate.recordMotion(1.02,.04,0);
    assertTrue(gate.canAim(1.03,.01,.04,.9));
    assertTrue(gate.captureStatus().contains("aligned"));
  }
  @Test void missingNativeHistoryAndSamplingGapsCannotCertifyACameraFrame() {
    var gate=new VisionShotGate();
    gate.recordMotion(1,0,0);gate.recordMotion(1.10,0,0);
    gate.recordCamera(1.05,14,4,.01,.05);
    assertFalse(gate.canAim(1.11,.01,0,.9));
    assertTrue(gate.captureStatus().contains("gapped"));
    gate.recordMotion(Double.NaN,0,0);
    assertFalse(gate.canScore(1.11,.01,0,0,.9));
  }
  @Test void captureTimeInterpolationAlsoWorksForNegativeFieldVelocity() {
    var gate=new VisionShotGate();
    for(int i=0;i<=80;i++) {
      double now=1+i*.01;
      gate.recordMotion(now,-.6,-.2);
      if(i>=10 && i%4==0) {
        double capture=now-.1;
        gate.recordCamera(capture,14-.6*(capture-1),4-.2*(capture-1),.02,.06);
      }
    }
    assertTrue(gate.canScore(1.81,.01,.64,.1,.9),gate.captureStatus());
  }

  @Test void aCarryNeverInventsAGyroRate() {
    var gate=cameraQualified(.6);
    for(int i=51;i<=90;i++) gate.recordMotion(1+i*.01,.6,0);
    assertTrue(gate.canScore(1.9,0,.6,0,.9));
    assertFalse(gate.canScore(1.9,0,.6,Double.NaN,.9));
    assertEquals("gyro rate unavailable",gate.scoreBlocker(1.9,0,.6,Double.NaN,.9));
  }

  @Test void theFieldIsConfigurableAndAFixOffItRevokesEvidence() {
    // The default field is 2026's; on a smaller one, a fix past its far wall is not a fix.
    var small=new VisionShotGate(new VisionShotGate.Config().field(8.0,4.5));
    for(int i=0;i<=6;i++) small.record(1+i*.05,7.5,4,.03,.07,0,0);
    assertTrue(small.canScore(1.32,.01,0,0,.9),"inside the configured field");
    small.record(1.35,8.5,4,.03,.07,0,0);
    assertFalse(small.canAim(1.36,.01,0,.9),"past the configured far wall");
    assertThrows(IllegalArgumentException.class,()->new VisionShotGate.Config().field(0,4));
    assertThrows(IllegalArgumentException.class,()->new VisionShotGate.Config().aimBudgetRad(.5));
    assertThrows(IllegalArgumentException.class,()->new VisionShotGate.Config().carry(-1,1));
  }

  @Test void aShorterCarryBudgetEndsTheCarrySooner() {
    var gate=new VisionShotGate(new VisionShotGate.Config().carry(.5,1.0));
    for(int i=0;i<=50;i++) {
      double now=1+i*.01;
      gate.recordMotion(now,0,0);
      if(i>=10 && i%5==0) gate.recordCamera(now-.08,9,4,.02,.06);
    }
    for(int i=51;i<=100;i++) gate.recordMotion(1+i*.01,0,0);
    assertTrue(gate.scoreBlocker(2.0,0,0,0,.9).contains("time budget"));
  }

  @Test void aWiderAimBudgetWidensTheAgreementLimitWithinItsClamp() {
    var gate=new VisionShotGate(new VisionShotGate.Config().aimBudgetRad(.05));
    gate.setTarget(10,4);
    assertEquals(.15,gate.agreementMeters(3),1e-12);
    assertEquals(.20,gate.agreementMeters(9),0);
    assertEquals(.10,gate.agreementMeters(1),0);
  }
}
