// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
class constants$11 {

    static final FunctionDescriptor NewtonIslandUpdate$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    static final MethodHandle NewtonIslandUpdate$MH = RuntimeHelper.downcallHandle(
        constants$11.NewtonIslandUpdate$FUNC, false
    );
    static final FunctionDescriptor NewtonFractureCompoundCollisionOnEmitCompoundFractured$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonFractureCompoundCollisionOnEmitCompoundFractured$MH = RuntimeHelper.downcallHandle(
        constants$11.NewtonFractureCompoundCollisionOnEmitCompoundFractured$FUNC, false
    );
    static final FunctionDescriptor NewtonFractureCompoundCollisionOnEmitChunk$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    static final MethodHandle NewtonFractureCompoundCollisionOnEmitChunk$MH = RuntimeHelper.downcallHandle(
        constants$11.NewtonFractureCompoundCollisionOnEmitChunk$FUNC, false
    );
}


