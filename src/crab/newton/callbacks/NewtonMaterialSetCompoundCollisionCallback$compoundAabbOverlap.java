// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * int (*NewtonMaterialSetCompoundCollisionCallback$compoundAabbOverlap)(struct NewtonJoint*,float,struct NewtonBody*,void*,struct NewtonBody*,void*,int);
 * }
 */
public interface NewtonMaterialSetCompoundCollisionCallback$compoundAabbOverlap {

    int apply(java.lang.foreign.MemorySegment contact, float timestep, java.lang.foreign.MemorySegment body0, java.lang.foreign.MemorySegment collisionNode0, java.lang.foreign.MemorySegment body1, java.lang.foreign.MemorySegment collisionNode1, int threadIndex);
    static MemorySegment allocate(NewtonMaterialSetCompoundCollisionCallback$compoundAabbOverlap fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$45.NewtonMaterialSetCompoundCollisionCallback$compoundAabbOverlap_UP$MH, fi, constants$45.NewtonMaterialSetCompoundCollisionCallback$compoundAabbOverlap$FUNC, scope);
    }
    static NewtonMaterialSetCompoundCollisionCallback$compoundAabbOverlap ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _contact, float _timestep, java.lang.foreign.MemorySegment _body0, java.lang.foreign.MemorySegment _collisionNode0, java.lang.foreign.MemorySegment _body1, java.lang.foreign.MemorySegment _collisionNode1, int _threadIndex) -> {
            try {
                return (int)constants$45.NewtonMaterialSetCompoundCollisionCallback$compoundAabbOverlap_DOWN$MH.invokeExact(symbol, _contact, _timestep, _body0, _collisionNode0, _body1, _collisionNode1, _threadIndex);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


