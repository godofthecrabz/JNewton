// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonWorldSetCollisionConstructorDestructorCallback$destructor)(struct NewtonWorld*,struct NewtonCollision*);
 * }
 */
public interface NewtonWorldSetCollisionConstructorDestructorCallback$destructor {

    void apply(java.lang.foreign.MemorySegment userJoint, java.lang.foreign.MemorySegment info);
    static MemorySegment allocate(NewtonWorldSetCollisionConstructorDestructorCallback$destructor fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$39.NewtonWorldSetCollisionConstructorDestructorCallback$destructor_UP$MH, fi, constants$39.NewtonWorldSetCollisionConstructorDestructorCallback$destructor$FUNC, scope);
    }
    static NewtonWorldSetCollisionConstructorDestructorCallback$destructor ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _userJoint, java.lang.foreign.MemorySegment _info) -> {
            try {
                constants$39.NewtonWorldSetCollisionConstructorDestructorCallback$destructor_DOWN$MH.invokeExact(symbol, _userJoint, _info);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}

