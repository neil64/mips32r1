


module MIPS32_Top
(
    input		CLK_I,
    input		RST_I,

    // Interrupts
    input [4:0]		INTR_I,
    input		NMI_I,

    // Instruction WB bus
    output [31:0]	I_ADR_O,
    output		I_CYC_O,
    output [31:0]	I_DAT_O,
    output [3:0]	I_SEL_O,
    output		I_STB_O,
    output		I_WE_O,
    output		I_LOCK_O,
    output [2:0]	I_CTI_O,
    output [1:0]	I_BTE_O,
    input		I_ACK_I,
    input [31:0]	I_DAT_I,
    input		I_RTY_I,
    input		I_ERR_I,

    // Data WB bus
    output [31:0]	D_ADR_O,
    output		D_CYC_O,
    output [31:0]	D_DAT_O,
    output [3:0]	D_SEL_O,
    output		D_STB_O,
    output		D_WE_O,
    output		D_LOCK_O,
    output [2:0]	D_CTI_O,
    output [1:0]	D_BTE_O,
    input		D_ACK_I,
    input [31:0]	D_DAT_I,
    input		D_RTY_I,
    input		D_ERR_I
);


    // Instruction local bus (between the I-cache and processor)
    wire [31:0]		iProcAddr;
    wire		iProcValid;
    wire		iProcStall;
    wire [31:0]		iProcDataIn;

    // Data local bus (between the D-cache and processor)
    wire [31:0]		dProcAddr;
    wire [31:0]		dProcDataOut;
    wire [3:0]		dProcLane;
    wire		dProcWrite;
    wire		dProcValid;
    wire		dProcStall;
    wire [31:0]		dProcDataIn;


    MIPS32_Processor MIPS32_Processor
    (
	.clock		(CLK_I),
	.reset		(RST_I),

	.Interrupts	(INTR_I),
	.NMI		(NMI_I),

	// Instruction local bus (to the I-cache)
	.Inst_Address	(iProcAddr),
	.Inst_Valid	(iProcValid),
	.Inst_Stall	(iProcStall),
	.Inst_In	(iProcDataIn),

	// Data local bus (to the D-cache)
	.Data_Address	(dProcAddr),
	.Data_Out	(dProcDataOut),
	.Data_Lane	(dProcLane),
	.Data_Write	(dProcWrite),
	.Data_Valid	(dProcValid),
	.Data_Stall	(dProcStall),
	.Data_In	(dProcDataIn)
    );


    MIPS32_ICache MIPS32_ICache
    (
	.clock		(CLK_I),
	.reset		(RST_I),

	// Instruction bus of the processor
	.Address	(iProcAddr),
	.Valid		(iProcValid),
	.Stall		(iProcStall),
	.In		(iProcDataIn),

	// External Wishbone memory bus
	.ADR_O		(I_ADR_O),
	.CYC_O		(I_CYC_O),
	.DAT_O		(I_DAT_O),
	.SEL_O		(I_SEL_O),
	.STB_O		(I_STB_O),
	.WE_O		(I_WE_O),
	.LOCK_O		(I_LOCK_O),
	.CTI_O		(I_CTI_O),
	.BTE_O		(I_BTE_O),
	.ACK_I		(I_ACK_I),
	.DAT_I		(I_DAT_I),
	.RTY_I		(I_RTY_I),
	.ERR_I		(I_ERR_I)
    );


    MIPS32_DCache MIPS32_DCache
    (
	.clock		(CLK_I),
	.reset		(RST_I),

	// Data bus of the processor
	.Address	(dProcAddr),
	.Out		(dProcDataOut),
	.Lane		(dProcLane),
	.Write		(dProcWrite),
	.Valid		(dProcValid),
	.Stall		(dProcStall),
	.In		(dProcDataIn)

	// External Wishbone memory bus
	.ADR_O		(D_ADR_O),
	.CYC_O		(D_CYC_O),
	.DAT_O		(D_DAT_O),
	.SEL_O		(D_SEL_O),
	.STB_O		(D_STB_O),
	.WE_O		(D_WE_O),
	.LOCK_O		(D_LOCK_O),
	.CTI_O		(D_CTI_O),
	.BTE_O		(D_BTE_O),
	.ACK_I		(D_ACK_I),
	.DAT_I		(D_DAT_I),
	.RTY_I		(D_RTY_I),
	.ERR_I		(D_ERR_I)
    );


endmodule
