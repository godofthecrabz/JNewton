// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$76 {

    static final FunctionDescriptor NewtonCreateDeformableSolid$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonCreateDeformableSolid$MH = RuntimeHelper.downcallHandle(
        "NewtonCreateDeformableSolid",
        constants$76.NewtonCreateDeformableSolid$FUNC, false
    );
    static final FunctionDescriptor NewtonDeformableMeshGetParticleCount$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonDeformableMeshGetParticleCount$MH = RuntimeHelper.downcallHandle(
        "NewtonDeformableMeshGetParticleCount",
        constants$76.NewtonDeformableMeshGetParticleCount$FUNC, false
    );
    static final FunctionDescriptor NewtonDeformableMeshGetParticleStrideInBytes$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonDeformableMeshGetParticleStrideInBytes$MH = RuntimeHelper.downcallHandle(
        "NewtonDeformableMeshGetParticleStrideInBytes",
        constants$76.NewtonDeformableMeshGetParticleStrideInBytes$FUNC, false
    );
    static final FunctionDescriptor NewtonDeformableMeshGetParticleArray$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonDeformableMeshGetParticleArray$MH = RuntimeHelper.downcallHandle(
        "NewtonDeformableMeshGetParticleArray",
        constants$76.NewtonDeformableMeshGetParticleArray$FUNC, false
    );
    static final FunctionDescriptor NewtonConstraintCreateBall$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonConstraintCreateBall$MH = RuntimeHelper.downcallHandle(
        "NewtonConstraintCreateBall",
        constants$76.NewtonConstraintCreateBall$FUNC, false
    );
    static final FunctionDescriptor NewtonBallSetUserCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonBallSetUserCallback$MH = RuntimeHelper.downcallHandle(
        "NewtonBallSetUserCallback",
        constants$76.NewtonBallSetUserCallback$FUNC, false
    );
}


