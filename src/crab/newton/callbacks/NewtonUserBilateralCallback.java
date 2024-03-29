// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonUserBilateralCallback)(struct NewtonJoint* userJoint,float timestep,int threadIndex);
 * }
 */
public interface NewtonUserBilateralCallback {

    void apply(java.lang.foreign.MemorySegment userJoint, float timestep, int threadIndex);
    static MemorySegment allocate(NewtonUserBilateralCallback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$17.NewtonUserBilateralCallback_UP$MH, fi, constants$17.NewtonUserBilateralCallback$FUNC, scope);
    }
    static NewtonUserBilateralCallback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _userJoint, float _timestep, int _threadIndex) -> {
            try {
                constants$17.NewtonUserBilateralCallback_DOWN$MH.invokeExact(symbol, _userJoint, _timestep, _threadIndex);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


