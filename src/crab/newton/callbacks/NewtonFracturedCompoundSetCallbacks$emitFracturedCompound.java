// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonFracturedCompoundSetCallbacks$emitFracturedCompound)(struct NewtonBody*);
 * }
 */
public interface NewtonFracturedCompoundSetCallbacks$emitFracturedCompound {

    void apply(java.lang.foreign.MemorySegment me);
    static MemorySegment allocate(NewtonFracturedCompoundSetCallbacks$emitFracturedCompound fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$59.NewtonFracturedCompoundSetCallbacks$emitFracturedCompound_UP$MH, fi, constants$59.NewtonFracturedCompoundSetCallbacks$emitFracturedCompound$FUNC, scope);
    }
    static NewtonFracturedCompoundSetCallbacks$emitFracturedCompound ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _me) -> {
            try {
                constants$59.NewtonFracturedCompoundSetCallbacks$emitFracturedCompound_DOWN$MH.invokeExact(symbol, _me);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


