// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonBodySetDestructorCallback$callback)(struct NewtonBody*);
 * }
 */
public interface NewtonBodySetDestructorCallback$callback {

    void apply(java.lang.foreign.MemorySegment me);
    static MemorySegment allocate(NewtonBodySetDestructorCallback$callback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$84.NewtonBodySetDestructorCallback$callback_UP$MH, fi, constants$84.NewtonBodySetDestructorCallback$callback$FUNC, scope);
    }
    static NewtonBodySetDestructorCallback$callback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _me) -> {
            try {
                constants$84.NewtonBodySetDestructorCallback$callback_DOWN$MH.invokeExact(symbol, _me);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}

