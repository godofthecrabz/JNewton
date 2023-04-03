// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonOnJointSerializationCallback)(struct NewtonJoint* joint,void (*function)(void*,void*,int),void* serializeHandle);
 * }
 */
public interface NewtonOnJointSerializationCallback {

    void apply(java.lang.foreign.MemorySegment body, java.lang.foreign.MemorySegment mainMesh, java.lang.foreign.MemorySegment fracturedCompountCollision);
    static MemorySegment allocate(NewtonOnJointSerializationCallback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$5.NewtonOnJointSerializationCallback_UP$MH, fi, constants$5.NewtonOnJointSerializationCallback$FUNC, scope);
    }
    static NewtonOnJointSerializationCallback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _body, java.lang.foreign.MemorySegment _mainMesh, java.lang.foreign.MemorySegment _fracturedCompountCollision) -> {
            try {
                constants$5.NewtonOnJointSerializationCallback_DOWN$MH.invokeExact(symbol, _body, _mainMesh, _fracturedCompountCollision);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


