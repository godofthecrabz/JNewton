// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonBodySetTransformCallback$callback)(struct NewtonBody*,float*,int);
 * }
 */
public interface NewtonBodySetTransformCallback$callback {

    void apply(java.lang.foreign.MemorySegment world, java.lang.foreign.MemorySegment userData, int threadIndex);
    static MemorySegment allocate(NewtonBodySetTransformCallback$callback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$85.NewtonBodySetTransformCallback$callback_UP$MH, fi, constants$85.NewtonBodySetTransformCallback$callback$FUNC, scope);
    }
    static NewtonBodySetTransformCallback$callback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _world, java.lang.foreign.MemorySegment _userData, int _threadIndex) -> {
            try {
                constants$85.NewtonBodySetTransformCallback$callback_DOWN$MH.invokeExact(symbol, _world, _userData, _threadIndex);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}

