// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonSerializeScene$bodyCallback)(struct NewtonBody*,void*,void (*)(void*,void*,int),void*);
 * }
 */
public interface NewtonSerializeScene$bodyCallback {

    void apply(java.lang.foreign.MemorySegment body0, java.lang.foreign.MemorySegment body1, java.lang.foreign.MemorySegment function, java.lang.foreign.MemorySegment serializeHandle);
    static MemorySegment allocate(NewtonSerializeScene$bodyCallback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$26.NewtonSerializeScene$bodyCallback_UP$MH, fi, constants$26.NewtonSerializeScene$bodyCallback$FUNC, scope);
    }
    static NewtonSerializeScene$bodyCallback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _body0, java.lang.foreign.MemorySegment _body1, java.lang.foreign.MemorySegment _function, java.lang.foreign.MemorySegment _serializeHandle) -> {
            try {
                constants$27.NewtonSerializeScene$bodyCallback_DOWN$MH.invokeExact(symbol, _body0, _body1, _function, _serializeHandle);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}

