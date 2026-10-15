package frc.lib.catalyst.physics.estimation;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import frc.lib.catalyst.physics.observation.PoseObservation;

/**
 * Original, optional recovery evidence from a sequence of externally qualified camera captures.
 * This is not a continuous pose filter or independent absolute-heading sensor. A quadratic local
 * time fit estimates ENDPOINT velocity, rather than mistaking interval-mean speed for current speed.
 * A fresh holdout capture is excluded from the fit. Uncertainty is model-based, not a calibrated
 * probability guarantee; shared calibration/gyro errors remain unobservable here.
 *
 * <p>Capture epochs, effective-information limits and reset cutoffs apply before fitting. The owner
 * must still compare the result with fresh native position AND wheel velocity before recovering.
 * No pose is written and no fault is cleared by this class. Calls must be serialized by the owner.
 */
public final class VisionPoseRecovery {
    private static final int MIN_TRAINING = 6, MAX_SAMPLES = 48;
    private static final double EFFECTIVE_SAMPLE_CAP = 16;
    private static final double MIN_SPAN = .25, MAX_WINDOW = 2.0;
    private static final double MIN_SPACING = .04, MAX_GAP = .16;
    private static final double MAX_AGE = .12;
    private static final double MAX_INPUT_STD = .08, SYSTEMATIC_FLOOR = .02;
    private static final double NOISE_INFLATION = 1.25;
    private static final double MAX_RESIDUAL = .10, MAX_HOLDOUT_ERROR = .08;
    private static final double MAX_VELOCITY_STD = .08;
    private static final double MAX_MODEL_ACCELERATION = 2.0;
    // Additional unmodelled endpoint-velocity uncertainty grows after the last camera capture.
    private static final double VELOCITY_PROCESS_MPS_PER_S = .5;
    private static final double PROJECTION_ACCELERATION_ALLOWANCE = 12.0;
    private final double maxSpeedMps;
    private final List<PoseObservation> samples = new ArrayList<>();
    private double faultEpoch = Double.NaN, cutoff = Double.NEGATIVE_INFINITY;
    private double lastSeenCapture = Double.NEGATIVE_INFINITY;
    private String source;
    private String reason = "no motion fault";
    private Fit pending;

    public VisionPoseRecovery(double maxSpeedMps) {
        if (!Double.isFinite(maxSpeedMps) || maxSpeedMps <= 0)
            throw new IllegalArgumentException("positive finite recovery speed limit required");
        this.maxSpeedMps = maxSpeedMps;
    }

    /** Immutable, current-time evidence; only this fitter can create an instance. */
    public static final class Evidence {
        private final Pose2d pose;
        private final Translation2d velocity;
        private final double timestamp, captureTimestamp, firstCapture, faultEpoch, validAfter;
        private final double positionStd, velocityStd;
        private final int samples;
        private Evidence(Pose2d pose, Translation2d velocity, double timestamp, double captureTimestamp,
                double firstCapture, double faultEpoch, double validAfter, double positionStd,
                double velocityStd, int samples) {
            this.pose=pose; this.velocity=velocity; this.timestamp=timestamp;
            this.captureTimestamp=captureTimestamp; this.firstCapture=firstCapture;
            this.faultEpoch=faultEpoch; this.validAfter=validAfter;
            this.positionStd=positionStd; this.velocityStd=velocityStd; this.samples=samples;
        }
        public Pose2d pose() { return pose; }
        public Translation2d fieldVelocity() { return velocity; }
        public double timestampSeconds() { return timestamp; }
        public double captureTimestampSeconds() { return captureTimestamp; }
        public double firstCaptureTimestampSeconds() { return firstCapture; }
        public double faultEpoch() { return faultEpoch; }
        public double validAfterTimestampSeconds() { return validAfter; }
        public double positionStdDevMeters() { return positionStd; }
        public double velocityStdDevMps() { return velocityStd; }
        public int sampleCount() { return samples; }
    }

    private record Fit(double[] x, double[] y, double[][] covariance, double residualScale,
            double referenceTime, double span, PoseObservation holdout, double inputStd,
            double firstCapture, int count) {}

    public String reason() { return reason; }
    public int sampleCount() { return samples.size(); }

    public void reset() {
        samples.clear(); pending=null; source=null;
        faultEpoch=Double.NaN; cutoff=lastSeenCapture=Double.NEGATIVE_INFINITY;
        reason="no motion fault";
    }

    /** A capture at/before this event may never repopulate the new evidence window. */
    public void invalidateBefore(double timestamp) {
        if (!Double.isFinite(timestamp)) return;
        cutoff=Math.max(cutoff,timestamp);
        samples.clear(); pending=null;
        reason="waiting for captures after reset/disturbance";
    }

    /** Input only after the caller's real camera-quality/field/heading checks have accepted it. */
    public boolean observe(PoseObservation observation, double now, double epoch) {
        if (!Double.isFinite(now) || !Double.isFinite(epoch)) return false;
        if (Double.compare(epoch,faultEpoch)!=0) {
            samples.clear(); pending=null; source=null;
            faultEpoch=epoch; cutoff=Math.max(cutoff,epoch);
        }
        if (observation==null || !validPose(observation.pose())
                || !finite(observation.timestampSeconds(),observation.standardDeviation())
                || observation.standardDeviation()<=0 || observation.standardDeviation()>MAX_INPUT_STD) {
            samples.clear(); pending=null; reason="invalid or uncertain camera capture"; return false;
        }
        double t=observation.timestampSeconds();
        if (t<=cutoff || t<=lastSeenCapture || t>now || now-t>MAX_AGE) {
            reason="old, duplicate, pre-event or future camera capture"; return false;
        }
        lastSeenCapture=t;
        if (source!=null && !source.equals(observation.source())) {
            samples.clear(); pending=null; reason="camera source changed; rebuilding evidence";
        }
        source=observation.source();
        if (!samples.isEmpty()) {
            double dt=t-samples.get(samples.size()-1).timestampSeconds();
            if (dt<MIN_SPACING-1e-9) return false; // More FPS is not more independent evidence.
            if (dt>MAX_GAP) { samples.clear(); pending=null; }
        }
        samples.add(observation);
        while (samples.size()>MAX_SAMPLES || t-samples.get(0).timestampSeconds()>MAX_WINDOW)
            samples.remove(0);
        pending=fit();
        return true;
    }

    /** Evaluates position, velocity and their uncertainty at the SAME current native capture time. */
    public Optional<Evidence> evidenceAt(double now, double epoch) {
        if (pending==null || Double.compare(epoch,faultEpoch)!=0 || !Double.isFinite(now))
            return Optional.empty();
        Fit f=pending;
        double age=now-f.holdout.timestampSeconds();
        if (age<0 || age>MAX_AGE) { reason="recovery holdout is stale or future"; return Optional.empty(); }
        double u=(now-f.referenceTime)/f.span;
        double uh=(f.holdout.timestampSeconds()-f.referenceTime)/f.span;
        double[] derivative={0,1/f.span,2*u/f.span};
        double[] second={0,0,2/(f.span*f.span)};
        Translation2d velocity=new Translation2d(dot(f.x,derivative),dot(f.y,derivative));
        Translation2d acceleration=new Translation2d(dot(f.x,second),dot(f.y,second));
        if (velocity.getNorm()>maxSpeedMps || acceleration.getNorm()>MAX_MODEL_ACCELERATION) {
            reason="motion exceeds recovery model envelope"; return Optional.empty();
        }
        double velocityStd=Math.sqrt(2*quadratic(f.covariance,derivative)*f.residualScale)
                + VELOCITY_PROCESS_MPS_PER_S*age;
        if (!Double.isFinite(velocityStd) || velocityStd>MAX_VELOCITY_STD) {
            reason="endpoint velocity uncertainty too high"; return Optional.empty();
        }
        double[] atHoldoutDerivative={0,1/f.span,2*uh/f.span};
        Translation2d captureVelocity=new Translation2d(dot(f.x,atHoldoutDerivative),dot(f.y,atHoldoutDerivative));
        Translation2d shift=captureVelocity.times(age).plus(acceleration.times(.5*age*age));
        double[] increment={0,u-uh,u*u-uh*uh};
        double positionStd=f.inputStd+Math.sqrt(2*quadratic(f.covariance,increment)*f.residualScale)
                + .5*PROJECTION_ACCELERATION_ALLOWANCE*age*age;
        if (!finite(positionStd,velocity.getX(),velocity.getY(),shift.getX(),shift.getY())) return Optional.empty();
        reason="validated camera sequence; awaiting native consistency";
        return Optional.of(new Evidence(new Pose2d(f.holdout.pose().getTranslation().plus(shift),
                f.holdout.pose().getRotation()),velocity,now,f.holdout.timestampSeconds(),
                f.firstCapture,faultEpoch,cutoff,positionStd,velocityStd,f.count));
    }

    private Fit fit() {
        int n=samples.size()-1; // The newest frame is strictly held out of model fitting.
        if (n<MIN_TRAINING) { reason="collecting post-fault camera captures"; return null; }
        double first=samples.get(0).timestampSeconds(), reference=samples.get(n-1).timestampSeconds();
        double span=reference-first;
        if (span<MIN_SPAN) { reason="camera evidence span too short"; return null; }
        double[][] normal=new double[3][3]; double[] bx=new double[3],by=new double[3];
        double informationScale=Math.min(1,EFFECTIVE_SAMPLE_CAP/n), inputStd=SYSTEMATIC_FLOOR;
        for (int k=0;k<n;k++) {
            var o=samples.get(k);
            double sigma=Math.max(SYSTEMATIC_FLOOR,o.standardDeviation());
            inputStd=Math.max(inputStd,sigma);
            double weight=informationScale/Math.pow(sigma*NOISE_INFLATION,2);
            double u=(o.timestampSeconds()-reference)/span; double[] basis={1,u,u*u};
            for(int i=0;i<3;i++) {
                bx[i]+=weight*basis[i]*o.pose().getX(); by[i]+=weight*basis[i]*o.pose().getY();
                for(int j=0;j<3;j++) normal[i][j]+=weight*basis[i]*basis[j];
            }
        }
        double[][] inverse=invert(normal);
        if(inverse==null) { reason="ill-conditioned recovery time fit"; return null; }
        double[] x=multiply(inverse,bx),y=multiply(inverse,by);
        double normalizedResidual=0;
        for(int k=0;k<n;k++) {
            var o=samples.get(k); double u=(o.timestampSeconds()-reference)/span;
            double error=Math.hypot(value(x,u)-o.pose().getX(),value(y,u)-o.pose().getY());
            double sigma=Math.max(SYSTEMATIC_FLOOR,o.standardDeviation());
            if(!Double.isFinite(error) || error>Math.min(MAX_RESIDUAL,3*sigma)) {
                reason="camera trajectory inconsistent"; return null;
            }
            normalizedResidual+=error*error/(sigma*sigma);
        }
        double residualScale=Math.max(1,normalizedResidual/Math.max(1,2*n-6));
        var holdout=samples.get(n); double u=(holdout.timestampSeconds()-reference)/span;
        double error=Math.hypot(value(x,u)-holdout.pose().getX(),value(y,u)-holdout.pose().getY());
        double holdoutSigma=Math.max(SYSTEMATIC_FLOOR,holdout.standardDeviation());
        if(!Double.isFinite(error) || error>Math.min(MAX_HOLDOUT_ERROR,3*holdoutSigma)) {
            reason="new camera capture disagrees with trajectory"; return null;
        }
        reason="camera fit ready; checking endpoint uncertainty";
        return new Fit(x,y,inverse,residualScale,reference,span,holdout,
                Math.max(inputStd,holdoutSigma),first,n+1);
    }

    private static double value(double[] c,double u) { return c[0]+u*c[1]+u*u*c[2]; }
    private static double dot(double[] a,double[] b) { double s=0;for(int i=0;i<a.length;i++)s+=a[i]*b[i];return s; }
    private static double[] multiply(double[][] a,double[] b) {
        double[] r=new double[3];for(int i=0;i<3;i++)for(int j=0;j<3;j++)r[i]+=a[i][j]*b[j];return r;
    }
    private static double quadratic(double[][] a,double[] b) { return Math.max(0,dot(b,multiply(a,b))); }
    /** Small original pivoted Gauss-Jordan solve on normalized-time equations; rejects lost rank. */
    private static double[][] invert(double[][] input) {
        double[][] a=new double[3][6]; double scale=0;
        for(int i=0;i<3;i++)for(int j=0;j<3;j++){a[i][j]=input[i][j];scale=Math.max(scale,Math.abs(input[i][j]));}
        if(!Double.isFinite(scale)||scale<=0)return null;
        for(int i=0;i<3;i++)a[i][i+3]=1;
        for(int col=0;col<3;col++) {
            int pivot=col;for(int r=col+1;r<3;r++)if(Math.abs(a[r][col])>Math.abs(a[pivot][col]))pivot=r;
            if(!Double.isFinite(a[pivot][col])||Math.abs(a[pivot][col])<scale*1e-10)return null;
            double[] swap=a[col];a[col]=a[pivot];a[pivot]=swap;
            double divisor=a[col][col];for(int j=0;j<6;j++)a[col][j]/=divisor;
            for(int r=0;r<3;r++)if(r!=col){double f=a[r][col];for(int j=0;j<6;j++)a[r][j]-=f*a[col][j];}
        }
        double[][] result=new double[3][3];
        for(int i=0;i<3;i++)for(int j=0;j<3;j++){result[i][j]=a[i][j+3];if(!Double.isFinite(result[i][j]))return null;}
        return result;
    }
    private static boolean validPose(Pose2d p) {
        return p!=null&&p.getTranslation()!=null&&p.getRotation()!=null
            &&finite(p.getX(),p.getY(),p.getRotation().getRadians());
    }
    private static boolean finite(double... numbers) {
        for(double n:numbers)if(!Double.isFinite(n))return false;return true;
    }
}
