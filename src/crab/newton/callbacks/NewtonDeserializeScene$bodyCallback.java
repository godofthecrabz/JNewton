// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonDeserializeScene$bodyCallback)(struct NewtonBody*,void*,void (*)(void*,void*,int),void*);
 * }
 */
public interface NewtonDeserializeScene$bodyCallback {

    void apply(java.lang.foreign.MemorySegment body0, java.lang.foreign.MemorySegment body1, java.lang.foreign.MemorySegment function, java.lang.foreign.MemorySegment serializeHandle);
    static MemorySegment allocate(NewtonDeserializeScene$bodyCallback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$27.NewtonDeserializeScene$bodyCallback_UP$MH, fi, constants$27.NewtonDeserializeScene$bodyCallback$FUNC, scope);
    }
    static NewtonDeserializeScene$bodyCallback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _body0, java.lang.foreign.MemorySegment _body1, java.lang.foreign.MemorySegment _function, java.lang.foreign.MemorySegment _serializeHandle) -> {
            try {
                constants$27.NewtonDeserializeScene$bodyCallback_DOWN$MH.invokeExact(symbol, _body0, _body1, _function, _serializeHandle);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


