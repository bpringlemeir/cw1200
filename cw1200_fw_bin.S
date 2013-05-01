    .section .rodata
    .align 4
_fw_wsm_22_bin:
    .incbin "wsm_22.bin"
	.globl	fw_wsm_22
fw_wsm_22:
        .word   fw_wsm_22 - _fw_wsm_22_bin   /* size */
        .word    _fw_wsm_22_bin              /* data */
        .word   0                            /* pages */
    .align 4
_fw_sdd_22_bin:
    .incbin "sdd_22.bin"
	.globl	fw_sdd_22
fw_sdd_22:
        .word   fw_sdd_22 - _fw_sdd_22_bin   /* size  */
        .word   _fw_sdd_22_bin               /* data  */
        .word   0                            /* pages */
