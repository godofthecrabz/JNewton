// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonWorldListenerGetBodyDestroyCallback$return)(struct NewtonWorld*,void*,struct NewtonBody*);
 * }
 */
public interface NewtonWorldListenerGetBodyDestroyCallback$return {

    void apply(java.lang.foreign.MemorySegment body, java.lang.foreign.MemorySegment mainMesh, java.lang.foreign.MemorySegment fracturedCompountCollision);
    static MemorySegment allocate(NewtonWorldListenerGetBodyDestroyCallback$return fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$37.NewtonWorldListenerGetBodyDestroyCallback$return_UP$MH, fi, constants$37.NewtonWorldListenerGetBodyDestroyCallback$return$FUNC, scope);
    }
    static NewtonWorldListenerGetBodyDestroyCallback$return ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _body, java.lang.foreign.MemorySegment _mainMesh, java.lang.foreign.MemorySegment _fracturedCompountCollision) -> {
            try {
                constants$38.NewtonWorldListenerGetBodyDestroyCallback$return_DOWN$MH.invokeExact(symbol, _body, _mainMesh, _fracturedCompountCollision);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


