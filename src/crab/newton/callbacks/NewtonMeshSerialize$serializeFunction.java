// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonMeshSerialize$serializeFunction)(void*,void*,int);
 * }
 */
public interface NewtonMeshSerialize$serializeFunction {

    void apply(java.lang.foreign.MemorySegment world, java.lang.foreign.MemorySegment userData, int threadIndex);
    static MemorySegment allocate(NewtonMeshSerialize$serializeFunction fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$112.NewtonMeshSerialize$serializeFunction_UP$MH, fi, constants$112.NewtonMeshSerialize$serializeFunction$FUNC, scope);
    }
    static NewtonMeshSerialize$serializeFunction ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _world, java.lang.foreign.MemorySegment _userData, int _threadIndex) -> {
            try {
                constants$112.NewtonMeshSerialize$serializeFunction_DOWN$MH.invokeExact(symbol, _world, _userData, _threadIndex);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}

