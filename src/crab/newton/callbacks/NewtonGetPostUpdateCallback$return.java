// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonGetPostUpdateCallback$return)(struct NewtonWorld*,float);
 * }
 */
public interface NewtonGetPostUpdateCallback$return {

    void apply(java.lang.foreign.MemorySegment ball, float timestep);
    static MemorySegment allocate(NewtonGetPostUpdateCallback$return fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$20.NewtonGetPostUpdateCallback$return_UP$MH, fi, constants$20.NewtonGetPostUpdateCallback$return$FUNC, scope);
    }
    static NewtonGetPostUpdateCallback$return ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _ball, float _timestep) -> {
            try {
                constants$20.NewtonGetPostUpdateCallback$return_DOWN$MH.invokeExact(symbol, _ball, _timestep);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


