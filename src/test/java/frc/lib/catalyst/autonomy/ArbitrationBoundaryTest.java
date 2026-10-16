package frc.lib.catalyst.autonomy;
import static org.junit.jupiter.api.Assertions.*;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;
class ArbitrationBoundaryTest {
    @Test void badPeriodCannotGrantInstantaneousAccelerationAndNextTickStillLimits() {
        for(double dt:new double[]{0,-.02,Double.NaN,Double.POSITIVE_INFINITY}) {
            var limiter=new VelocityLimiter();limiter.reset(1,0);
            var held=limiter.limit(dt,-4,0,1,2);
            assertEquals(1,held[0],1e-12,"no trusted elapsed time means no finite-rate velocity change");
            assertEquals(.96,limiter.limit(.02,-4,0,1,2)[0],1e-12);
        }
    }
    @Test void invalidScoreCannotOwnResourcesAheadOfEligibleFiniteTask() {
        var bad=new TaskArbiter.Candidate<>("bad","bad",Double.NaN,Set.of("drive"),true);
        var good=new TaskArbiter.Candidate<>("good","good",.8,Set.of("drive"),true);
        for(var list:List.of(List.of(bad,good),List.of(good,bad))) {
            var selected=TaskArbiter.select(list,.5);
            assertEquals(List.of("good"),selected.tasks());
            assertEquals(1,selected.skipped().size());assertEquals("bad",selected.skipped().getFirst().name());
        }
    }
}
