// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonDispachThreadJob$task)(struct NewtonWorld*,void*,int);
 * }
 */
public interface NewtonDispachThreadJob$task {

    void apply(java.lang.foreign.MemorySegment world, java.lang.foreign.MemorySegment userData, int threadIndex);
    static MemorySegment allocate(NewtonDispachThreadJob$task fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$31.NewtonDispachThreadJob$task_UP$MH, fi, constants$31.NewtonDispachThreadJob$task$FUNC, scope);
    }
    static NewtonDispachThreadJob$task ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _world, java.lang.foreign.MemorySegment _userData, int _threadIndex) -> {
            try {
                constants$31.NewtonDispachThreadJob$task_DOWN$MH.invokeExact(symbol, _world, _userData, _threadIndex);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


