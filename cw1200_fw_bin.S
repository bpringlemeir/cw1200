        .section .rodata

         /* Raw data */
        .align  2
_fw_wsm_22_bin:
        .incbin "wsm_22.bin"
1:

         /* A firmware struct for wsm_22 */
        .align  2
	.globl	fw_wsm_22
fw_wsm_22:
        .word   1b - _fw_wsm_22_bin   /* size */
        .word    _fw_wsm_22_bin       /* data */
        .word   0                     /* pages */

         /* Raw data */
        .align  2
_fw_sdd_22_bin:
/*	.incbin "sdd_22.bin"*/
	.incbin "sdd_sagrad_1091_1098.bin"
1:

         /* A firmware struct for sdd_22 */
        .align  2
	.globl	fw_sdd_22
fw_sdd_22:
        .word   1b - _fw_sdd_22_bin   /* size  */
        .word   _fw_sdd_22_bin        /* data  */
        .word   0                     /* pages */
