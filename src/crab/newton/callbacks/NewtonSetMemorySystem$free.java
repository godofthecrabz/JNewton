// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonSetMemorySystem$free)(void*,int);
 * }
 */
public interface NewtonSetMemorySystem$free {

    void apply(java.lang.foreign.MemorySegment ptr, int sizeInBytes);
    static MemorySegment allocate(NewtonSetMemorySystem$free fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$19.NewtonSetMemorySystem$free_UP$MH, fi, constants$19.NewtonSetMemorySystem$free$FUNC, scope);
    }
    static NewtonSetMemorySystem$free ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _ptr, int _sizeInBytes) -> {
            try {
                constants$19.NewtonSetMemorySystem$free_DOWN$MH.invokeExact(symbol, _ptr, _sizeInBytes);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


