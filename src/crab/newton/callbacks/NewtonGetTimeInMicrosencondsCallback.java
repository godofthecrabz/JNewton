// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * long long (*NewtonGetTimeInMicrosencondsCallback)();
 * }
 */
public interface NewtonGetTimeInMicrosencondsCallback {

    long apply();
    static MemorySegment allocate(NewtonGetTimeInMicrosencondsCallback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$3.NewtonGetTimeInMicrosencondsCallback_UP$MH, fi, constants$3.NewtonGetTimeInMicrosencondsCallback$FUNC, scope);
    }
    static NewtonGetTimeInMicrosencondsCallback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return () -> {
            try {
                return (long)constants$3.NewtonGetTimeInMicrosencondsCallback_DOWN$MH.invokeExact(symbol);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


