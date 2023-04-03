// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonUserJointSetFeedbackCollectorCallback$getFeedback)(struct NewtonJoint*,float,int);
 * }
 */
public interface NewtonUserJointSetFeedbackCollectorCallback$getFeedback {

    void apply(java.lang.foreign.MemorySegment userJoint, float timestep, int threadIndex);
    static MemorySegment allocate(NewtonUserJointSetFeedbackCollectorCallback$getFeedback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$107.NewtonUserJointSetFeedbackCollectorCallback$getFeedback_UP$MH, fi, constants$107.NewtonUserJointSetFeedbackCollectorCallback$getFeedback$FUNC, scope);
    }
    static NewtonUserJointSetFeedbackCollectorCallback$getFeedback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _userJoint, float _timestep, int _threadIndex) -> {
            try {
                constants$108.NewtonUserJointSetFeedbackCollectorCallback$getFeedback_DOWN$MH.invokeExact(symbol, _userJoint, _timestep, _threadIndex);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


