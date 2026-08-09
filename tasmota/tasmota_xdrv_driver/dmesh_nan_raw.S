/*
 * Xtensa-LX106 ABI bridge for NONOSDK22x_190703.
 *
 * The SDK's ieee80211_freedom_output() has a 32-byte call frame.  The public
 * wrapper rejects NAN action frames before function offset +0x68; this bridge creates
 * the same frame and jumps directly to that allocation/transmit tail.  The
 * SDK function restores the saved registers and returns through our caller.
 */

#ifdef ESP8266

    .section .text.dmesh_nan_raw_trampoline,"ax",@progbits
    .align 4
    .global dmesh_nan_raw_trampoline
    .type dmesh_nan_raw_trampoline, @function
dmesh_nan_raw_trampoline:
    /* Arguments: a2=freedom_output, a3=conn, a4=frame, a5=len, a6=sys_seq */
    addi    a1, a1, -32
    s32i.n  a0, a1, 12
    s32i.n  a12, a1, 16
    s32i.n  a13, a1, 20
    s32i.n  a14, a1, 24
    s32i.n  a3, a1, 0
    s32i.n  a6, a1, 4
    s32i.n  a3, a1, 28
    mov.n   a14, a3
    mov.n   a13, a4
    mov.n   a12, a5
    addi    a0, a2, 0x68
    jx      a0
    .size dmesh_nan_raw_trampoline, .-dmesh_nan_raw_trampoline

#endif
