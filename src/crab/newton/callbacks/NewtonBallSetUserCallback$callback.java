// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonBallSetUserCallback$callback)(struct NewtonJoint*,float);
 * }
 */
public interface NewtonBallSetUserCallback$callback {

    void apply(java.lang.foreign.MemorySegment ball, float timestep);
    static MemorySegment allocate(NewtonBallSetUserCallback$callback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$98.NewtonBallSetUserCallback$callback_UP$MH, fi, constants$98.NewtonBallSetUserCallback$callback$FUNC, scope);
    }
    static NewtonBallSetUserCallback$callback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _ball, float _timestep) -> {
            try {
                constants$99.NewtonBallSetUserCallback$callback_DOWN$MH.invokeExact(symbol, _ball, _timestep);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


