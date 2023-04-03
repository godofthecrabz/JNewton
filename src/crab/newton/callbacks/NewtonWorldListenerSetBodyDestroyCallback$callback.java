// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonWorldListenerSetBodyDestroyCallback$callback)(struct NewtonWorld*,void*,struct NewtonBody*);
 * }
 */
public interface NewtonWorldListenerSetBodyDestroyCallback$callback {

    void apply(java.lang.foreign.MemorySegment body, java.lang.foreign.MemorySegment mainMesh, java.lang.foreign.MemorySegment fracturedCompountCollision);
    static MemorySegment allocate(NewtonWorldListenerSetBodyDestroyCallback$callback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$37.NewtonWorldListenerSetBodyDestroyCallback$callback_UP$MH, fi, constants$37.NewtonWorldListenerSetBodyDestroyCallback$callback$FUNC, scope);
    }
    static NewtonWorldListenerSetBodyDestroyCallback$callback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _body, java.lang.foreign.MemorySegment _mainMesh, java.lang.foreign.MemorySegment _fracturedCompountCollision) -> {
            try {
                constants$37.NewtonWorldListenerSetBodyDestroyCallback$callback_DOWN$MH.invokeExact(symbol, _body, _mainMesh, _fracturedCompountCollision);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


