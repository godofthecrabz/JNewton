// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonFreeMemory)(void* ptr,int sizeInBytes);
 * }
 */
public interface NewtonFreeMemory {

    void apply(java.lang.foreign.MemorySegment ptr, int sizeInBytes);
    static MemorySegment allocate(NewtonFreeMemory fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$0.NewtonFreeMemory_UP$MH, fi, constants$0.NewtonFreeMemory$FUNC, scope);
    }
    static NewtonFreeMemory ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _ptr, int _sizeInBytes) -> {
            try {
                constants$0.NewtonFreeMemory_DOWN$MH.invokeExact(symbol, _ptr, _sizeInBytes);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


