// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonWorldSetCreateDestroyContactCallback$destroyContact)(struct NewtonWorld*,struct NewtonJoint*);
 * }
 */
public interface NewtonWorldSetCreateDestroyContactCallback$destroyContact {

    void apply(java.lang.foreign.MemorySegment userJoint, java.lang.foreign.MemorySegment info);
    static MemorySegment allocate(NewtonWorldSetCreateDestroyContactCallback$destroyContact fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$40.NewtonWorldSetCreateDestroyContactCallback$destroyContact_UP$MH, fi, constants$40.NewtonWorldSetCreateDestroyContactCallback$destroyContact$FUNC, scope);
    }
    static NewtonWorldSetCreateDestroyContactCallback$destroyContact ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _userJoint, java.lang.foreign.MemorySegment _info) -> {
            try {
                constants$40.NewtonWorldSetCreateDestroyContactCallback$destroyContact_DOWN$MH.invokeExact(symbol, _userJoint, _info);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


