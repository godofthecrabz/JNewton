// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonWorldDestroyListenerCallback)(struct NewtonWorld* world,void* listenerUserData);
 * }
 */
public interface NewtonWorldDestroyListenerCallback {

    void apply(java.lang.foreign.MemorySegment userJoint, java.lang.foreign.MemorySegment info);
    static MemorySegment allocate(NewtonWorldDestroyListenerCallback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$3.NewtonWorldDestroyListenerCallback_UP$MH, fi, constants$3.NewtonWorldDestroyListenerCallback$FUNC, scope);
    }
    static NewtonWorldDestroyListenerCallback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _userJoint, java.lang.foreign.MemorySegment _info) -> {
            try {
                constants$3.NewtonWorldDestroyListenerCallback_DOWN$MH.invokeExact(symbol, _userJoint, _info);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


