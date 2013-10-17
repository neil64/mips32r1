`timescale 1ns / 1ps

/*
 *  A wrapper for block RAM.
 *
 *  Carefully arrange verilog so that the symthesis tools will infer
 *  block RAM of the correct shape.
 */


module MIPS32_RAM
#(
    parameter	AWIDTH = 4,	    // Address width
		DWIDTH = 8,	    // Data width
		LANES = 1	    // Byte lanes (1 or more)
)
(
    input		    clock,
    input		    reset,

    input [AWIDTH-1 : 0]    readAddr,
    output [DWIDTH-1 : 0]   readData,

    input [AWIDTH-1 : 0]    writeAddr,
    input [DWIDTH-1 : 0]    writeData,
    input [LANES-1 : 0]	    writeLane,
    input		    writeEnable
);


    // The RAM and the registered read address
    reg [DWIDTH-1 : 0]	mem[0 : (1<<AWIDTH)-1];
    reg [AWIDTH-1 : 0]	ra;

    // The read side
    always @(posedge clock)
	ra <= readAddr;
    assign readData = mem[ra];


    // The write side
    genvar i;
    generate

	if (LANES == 1)
	begin

	    // Write the entire word in one operation
	    always @(posedge clock)
		if (writeEnable)
		    mem[writeAddr] <= writeData;

	end else begin

	    // Write each byte, if allowed
	    for (i = 0; i < DWIDTH/8; i = i + 1)
	    begin : wr
		always @(posedge clock)
		    if (writeLane[i])
			mem[writeAddr][(i*8)+7 : (i*8)] <=
						writeData[(i*8)+7 : (i*8)];
	    end

	end

    endgenerate


endmodule



module MIPS32_DPRAM
#(
    parameter	AWIDTH = 4,	    // Address width
		DWIDTH = 8,	    // Data width
		LANES = 1	    // Byte lanes (1 or more)
)
(
    input		    clock,
    input		    reset,

    input [AWIDTH-1 : 0]    readAddrA,
    output [DWIDTH-1 : 0]   readDataA,

    input [AWIDTH-1 : 0]    writeAddrA,
    input [DWIDTH-1 : 0]    writeDataA,
    input [LANES-1 : 0]	    writeLaneA,
    input		    writeEnableA,

    input [AWIDTH-1 : 0]    readAddrB,
    output [DWIDTH-1 : 0]   readDataB,

    input [AWIDTH-1 : 0]    writeAddrB,
    input [DWIDTH-1 : 0]    writeDataB,
    input [LANES-1 : 0]	    writeLaneB,
    input		    writeEnableB
);


    // The RAM and the registered read addresses
    reg [DWIDTH-1 : 0]	mem[0 : (1<<AWIDTH)-1];
    reg [AWIDTH-1 : 0]	raA, raB;

    // The read side (A)
    always @(posedge clock)
	raA <= readAddrA;
    assign readDataA = mem[raA];

    // The read side (B)
    always @(posedge clock)
	raB <= readAddrB;
    assign readDataB = mem[raB];


    // The write side
    genvar i;
    generate

	if (LANES == 1)
	begin

	    // Write the entire word in one operation (A)
	    always @(posedge clock)
		if (writeEnableA)
		    mem[writeAddrA] <= writeDataA;

	    // Write the entire word in one operation (B)
	    always @(posedge clock)
		if (writeEnableB)
		    mem[writeAddrB] <= writeDataB;

	end else begin

	    // Write each byte, if allowed
	    for (i = 0; i < DWIDTH/8; i = i + 1)
	    begin : wr

		always @(posedge clock)
		    if (writeLaneA[i])
			mem[writeAddrA][(i*8)+7 : (i*8)] <=
						writeDataA[(i*8)+7 : (i*8)];

		always @(posedge clock)
		    if (writeLaneB[i])
			mem[writeAddrB][(i*8)+7 : (i*8)] <=
						writeDataB[(i*8)+7 : (i*8)];
	    end

	end

    endgenerate


endmodule

