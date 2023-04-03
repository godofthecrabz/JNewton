// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonDestroyContactCallback)(struct NewtonWorld* newtonWorld,struct NewtonJoint* contact);
 * }
 */
public interface NewtonDestroyContactCallback {

    void apply(java.lang.foreign.MemorySegment userJoint, java.lang.foreign.MemorySegment info);
    static MemorySegment allocate(NewtonDestroyContactCallback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$1.NewtonDestroyContactCallback_UP$MH, fi, constants$1.NewtonDestroyContactCallback$FUNC, scope);
    }
    static NewtonDestroyContactCallback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _userJoint, java.lang.foreign.MemorySegment _info) -> {
            try {
                constants$1.NewtonDestroyContactCallback_DOWN$MH.invokeExact(symbol, _userJoint, _info);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


