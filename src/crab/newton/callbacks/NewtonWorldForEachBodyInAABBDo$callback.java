// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * int (*NewtonWorldForEachBodyInAABBDo$callback)(struct NewtonBody*,void*);
 * }
 */
public interface NewtonWorldForEachBodyInAABBDo$callback {

    int apply(java.lang.foreign.MemorySegment body, java.lang.foreign.MemorySegment userData);
    static MemorySegment allocate(NewtonWorldForEachBodyInAABBDo$callback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$33.NewtonWorldForEachBodyInAABBDo$callback_UP$MH, fi, constants$33.NewtonWorldForEachBodyInAABBDo$callback$FUNC, scope);
    }
    static NewtonWorldForEachBodyInAABBDo$callback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _body, java.lang.foreign.MemorySegment _userData) -> {
            try {
                return (int)constants$33.NewtonWorldForEachBodyInAABBDo$callback_DOWN$MH.invokeExact(symbol, _body, _userData);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


