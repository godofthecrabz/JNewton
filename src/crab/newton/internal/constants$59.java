// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$59 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$59() {}
    public static final FunctionDescriptor NewtonFracturedCompoundSetCallbacks$emitFracturedCompound$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonFracturedCompoundSetCallbacks$emitFracturedCompound_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonFracturedCompoundSetCallbacks$emitFracturedCompound_UP$MH = RuntimeHelper.upcallHandle(NewtonFracturedCompoundSetCallbacks$emitFracturedCompound.class, "apply", constants$59.NewtonFracturedCompoundSetCallbacks$emitFracturedCompound_UP$FUNC);
    public static final FunctionDescriptor NewtonFracturedCompoundSetCallbacks$emitFracturedCompound_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonFracturedCompoundSetCallbacks$emitFracturedCompound_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$59.NewtonFracturedCompoundSetCallbacks$emitFracturedCompound_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonFracturedCompoundSetCallbacks$emitFracfuredChunk$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonFracturedCompoundSetCallbacks$emitFracfuredChunk_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonFracturedCompoundSetCallbacks$emitFracfuredChunk_UP$MH = RuntimeHelper.upcallHandle(NewtonFracturedCompoundSetCallbacks$emitFracfuredChunk.class, "apply", constants$59.NewtonFracturedCompoundSetCallbacks$emitFracfuredChunk_UP$FUNC);
    public static final FunctionDescriptor NewtonFracturedCompoundSetCallbacks$emitFracfuredChunk_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonFracturedCompoundSetCallbacks$emitFracfuredChunk_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$59.NewtonFracturedCompoundSetCallbacks$emitFracfuredChunk_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonFracturedCompoundSetCallbacks$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonFracturedCompoundSetCallbacks$MH = RuntimeHelper.downcallHandle(
        "NewtonFracturedCompoundSetCallbacks",
        constants$59.NewtonFracturedCompoundSetCallbacks$FUNC
    );
    public static final FunctionDescriptor NewtonFracturedCompoundIsNodeFreeToDetach$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonFracturedCompoundIsNodeFreeToDetach$MH = RuntimeHelper.downcallHandle(
        "NewtonFracturedCompoundIsNodeFreeToDetach",
        constants$59.NewtonFracturedCompoundIsNodeFreeToDetach$FUNC
    );
}


