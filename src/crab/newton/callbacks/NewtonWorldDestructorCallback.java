// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonWorldDestructorCallback)(struct NewtonWorld* world);
 * }
 */
public interface NewtonWorldDestructorCallback {

    void apply(java.lang.foreign.MemorySegment me);
    static MemorySegment allocate(NewtonWorldDestructorCallback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$0.NewtonWorldDestructorCallback_UP$MH, fi, constants$0.NewtonWorldDestructorCallback$FUNC, scope);
    }
    static NewtonWorldDestructorCallback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _me) -> {
            try {
                constants$0.NewtonWorldDestructorCallback_DOWN$MH.invokeExact(symbol, _me);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


