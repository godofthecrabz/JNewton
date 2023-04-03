// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonWorldUpdateListenerCallback)(struct NewtonWorld* world,void* listenerUserData,float timestep);
 * }
 */
public interface NewtonWorldUpdateListenerCallback {

    void apply(java.lang.foreign.MemorySegment world, java.lang.foreign.MemorySegment listenerUserData, float timestep);
    static MemorySegment allocate(NewtonWorldUpdateListenerCallback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$2.NewtonWorldUpdateListenerCallback_UP$MH, fi, constants$2.NewtonWorldUpdateListenerCallback$FUNC, scope);
    }
    static NewtonWorldUpdateListenerCallback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _world, java.lang.foreign.MemorySegment _listenerUserData, float _timestep) -> {
            try {
                constants$2.NewtonWorldUpdateListenerCallback_DOWN$MH.invokeExact(symbol, _world, _listenerUserData, _timestep);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


