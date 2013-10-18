`timescale 1ns / 1ps
/*
 * File         : RegisterFile.v
 * Project      : University of Utah, XUM Project MIPS32 core
 * Creator(s)   : Grant Ayers (ayers@cs.utah.edu)
 *
 * Modification History:
 *   Rev   Date         Initials  Description of Change
 *   1.0   7-Jun-2011   GEA       Initial design.
 *
 * Standards/Formatting:
 *   Verilog 2001, 4 soft tab, wide column.
 *
 * Description:
 *   A Register File for a MIPS processor. Contains 32 general-purpose
 *   32-bit wide registers and two read ports. Register 0 always reads
 *   as zero.
 */
module RegisterFile(
    input  clock,
    input  reset,
    input  [4:0]  ReadReg1, ReadReg2, WriteReg,
    input  [4:0]  IF_ReadReg1, IF_ReadReg2,
    input  [31:0] WriteData,
    input  RegWrite,
    output [31:0] ReadData1, ReadData2
    );

`define DISTRAM
// `define CLOCKED_NEG_REGS
// `define CLOCKED_REGS


`ifdef DISTRAM


// (* ram_style = "distributed" *) reg [DATA-1:0] mem[(2**ADDR)-1:0];

// always @(posedge clk) begin
// rd0_data    <= mem[rd0_addr];
// rd1_data    <= mem[rd1_addr];
// if(wr_en) begin
// mem[wr_addr]    <= wr_data;
// end
// end

// endmodule


    // Register file of 32 32-bit registers.  Hopefully inferred to a
    // RAM32x2Q -- quad port distributed RAM (RAM in LUTs).
    (* ram_style = "distributed" *) reg [31:0] registers[0:31];

    // Sequential (clocked) write.
    // 'WriteReg' is the register index to write. 'RegWrite' is the command.
    always @(posedge clock)
        if (RegWrite)
            registers[WriteReg] <= WriteData;

    // Combinatorial Read. Register 0 is all 0s.
    assign ReadData1 = (ReadReg1 == 0) ? 32'h00000000 : registers[ReadReg1];
    assign ReadData2 = (ReadReg2 == 0) ? 32'h00000000 : registers[ReadReg2];

`else // DISTRAM
`ifdef CLOCKED_NEG_REGS

    // Register file of 32 32-bit registers.  Hopefully inferred to a
    // dual ported RAM.  The pipelined register addresses fit the model
    // of block RAM.
    reg [31:0] registers[0:31];
    reg [4:0]   reg1, reg2;

    // Sequential (clocked) write.
    // 'WriteReg' is the register index to write. 'RegWrite' is the command.
    always @(posedge clock)
        if (RegWrite)
            registers[WriteReg] <= WriteData;

    // Pipeline the read register addresses.
    always @(negedge clock) begin
        reg1 <= ReadReg1;
        reg2 <= ReadReg2;
    end

    // The read. Register 0 is all 0s.
    assign ReadData1 = (reg1 == 0) ? 32'h00000000 : registers[reg1];
    assign ReadData2 = (reg2 == 0) ? 32'h00000000 : registers[reg2];

`else // CLOCKED_NEG_REGS
`ifdef CLOCKED_REGS

    // Register file of 32 32-bit registers.  Hopefully inferred to a
    // dual ported RAM.  The pipelined register addresses fit the model
    // of block RAM.
    reg [31:0] registers[0:31];
    reg [4:0]   reg1, reg2;

    // Sequential (clocked) write.
    // 'WriteReg' is the register index to write. 'RegWrite' is the command.
    always @(posedge clock)
        if (RegWrite)
            registers[WriteReg] <= WriteData;

    // Pipeline the read register addresses.
    always @(posedge clock) begin
        reg1 <= IF_ReadReg1;
        reg2 <= IF_ReadReg2;
    end

    // The read. Register 0 is all 0s.
    assign ReadData1 = (reg1 == 0) ? 32'h00000000 : registers[reg1];
    assign ReadData2 = (reg2 == 0) ? 32'h00000000 : registers[reg2];

`else // CLOCKED_REGS

    // Register file of 32 32-bit registers. Register 0 is hardwired to 0s
    reg [31:0] registers [1:31];

    // Initialize all to zero
    integer i;
    initial begin
        for (i=1; i<32; i=i+1) begin
            registers[i] <= 0;
        end
    end

    // Sequential (clocked) write.
    // 'WriteReg' is the register index to write. 'RegWrite' is the command.
    always @(posedge clock) begin
        if (reset) begin
            for (i=1; i<32; i=i+1) begin
                registers[i] <= 0;
            end
        end
        else begin
            if (WriteReg != 0)
                registers[WriteReg] <= (RegWrite) ? WriteData : registers[WriteReg];
        end
    end

    // Combinatorial Read. Register 0 is all 0s.
    assign ReadData1 = (ReadReg1 == 0) ? 32'h00000000 : registers[ReadReg1];
    assign ReadData2 = (ReadReg2 == 0) ? 32'h00000000 : registers[ReadReg2];

`endif // CLOCKED_REGS
`endif // CLOCKED_NEG_REGS
`endif // DISTRAM

endmodule


// vim:set expandtab shiftwidth=4 softtabstop=4 syntax=verilog:
