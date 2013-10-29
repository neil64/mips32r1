`timescale 1ns / 1ps

/*
 *  A wrapper for block RAM.
 *
 *  Carefully arrange verilog so that the symthesis tools will infer
 *  block RAM of the correct shape.
 */

// `define XILINX


/*
 *	Block RAM instantiation, just general enough to handle the needs
 *	of the MIPS32 CPU's cache.
 */

module MIPS32_Cache_RAM
#(
    parameter	AWIDTH = 8,	        // Address width
                DWIDTH = 8,             // Data width
                LANES = (DWIDTH+7)/8    // (byte lanes)
)
(
    input		    clock,
    input		    reset,

    input [AWIDTH-1 : 0]    readAddr,
    input		    readEnable,
    output [DWIDTH-1 : 0]   readData,

    input [AWIDTH-1 : 0]    writeAddr,
    input [DWIDTH-1 : 0]    writeData,
    input [LANES-1 : 0]	    writeLane,
    input		    writeEnable
);

    /****************************************/

`ifdef XILINX

    XilinxBlockRAM
    #(
        .AWIDTH(AWIDTH),
        .DWIDTH(DWIDTH)
    )
    ram
    (
        .clk                (clock),
        .rst                (reset),

        .p0_addr            (readAddr),
        .p0_en              (readEnable),
        .p0_wen             ({LANES{1'b0}}),
        .p0_wdata           ({DWIDTH{1'b0}}),
        .p0_rdata           (readData),

        .p1_addr            (writeAddr),
        .p1_en              (writeEnable),
        .p1_wen             (writeLane),
        .p1_wdata           (writeData),
        .p1_rdata           ()
    );

`else // XILINX

    // The RAM and the registered read address
    reg [DWIDTH-1 : 0]	mem[0 : (1<<AWIDTH)-1];
    reg [AWIDTH-1 : 0]	ra;

    // The bypass;  this avoids the write/read contention that can occur in
    // some FPGA block RAM (like the Xilinx), and in some cases can speed
    // things up by a cycle.
    wire bypass = (writeAddr == ra && writeEnable);


    // The read side
    always @(posedge clock)
	if (readEnable)
	    ra <= readAddr;
    assign readData = bypass ? writeData : mem[ra];


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
	    for (i = 0; i < LANES; i = i + 1)
	    begin : wr
		always @(posedge clock)
		    if (writeEnable && writeLane[i])
			mem[writeAddr][(i*8)+7 : (i*8)] <=
						writeData[(i*8)+7 : (i*8)];
	    end

	end

    endgenerate

`endif // XILINX

endmodule



`ifdef old_way

/*
 *  The old way.  I prefer this method, since it describes what is needed
 *  very well, letting the tools figure out the best way.  But it doesn't
 *  word correctly with the Xilinx tools, because of a long standing bug.
 *  It seems like not all of the address lines are connected.
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
    input		    readEnable,
    output [DWIDTH-1 : 0]   readData,

    input [AWIDTH-1 : 0]    writeAddr,
    input [DWIDTH-1 : 0]    writeData,
    input [LANES-1 : 0]	    writeLane,
    input		    writeEnable
);

    // The RAM and the registered read address
    reg [DWIDTH-1 : 0]	mem[0 : (1<<AWIDTH)-1];
    reg [AWIDTH-1 : 0]	ra;


    // The bypass
//    reg [DWIDTH-1 : 0]  bypass;
//    reg                 bypassed;
//    always @(posedge clock)
//	if (reset)
//	    bypassed <= 1'b0;
//	else if (readEnable && writeEnable && readAddr == writeAddr)
//	begin
//	    bypassed <= 1'b1;
//	    bypass <= writeData;
//	end
//	else if (readEnable || writeEnable)
//	    bypassed <= 1'b0;


    wire [DWIDTH-1 : 0]	bypass = writeData;
    wire		bypassed = (writeAddr == ra && writeEnable);


    // The read side
    always @(posedge clock)
	if (readEnable)
	    ra <= readAddr;
    assign readData = bypassed ? bypass : mem[ra];


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

`endif // old_way


/**********************************************************************/
/**********************************************************************/
/**********************************************************************/
/**********************************************************************/
/**********************************************************************/
/**********************************************************************/

`ifdef crap2

/*
 *	A wrapper for the Xilinx block RAM.
 */

module XilinxBlockRAM
#(
    parameter	AWIDTH = 9,	        // Address width
                DWIDTH = 22,             // Data width
                LANES = (DWIDTH+7)/8    // (byte lanes)
)
(
    input		    clk,
    input		    rst,

    input [AWIDTH-1 : 0]    p0_addr,
    input		    p0_en,
    input [LANES-1 : 0]     p0_wen,
    input [DWIDTH-1 : 0]    p0_wdata,
    output [DWIDTH-1 : 0]   p0_rdata,

    input [AWIDTH-1 : 0]    p1_addr,
    input		    p1_en,
    input [LANES-1 : 0]     p1_wen,
    input [DWIDTH-1 : 0]    p1_wdata,
    output [DWIDTH-1 : 0]   p1_rdata
);

    /****************************************/

`ifdef XILINX

    // The width of the bank field
    localparam  BWIDTH = (DWIDTH ==  1) ? ((AWIDTH > 14) ? AWIDTH-14 : 0) :
                         (DWIDTH ==  2) ? ((AWIDTH > 13) ? AWIDTH-13 : 0) :
                         (DWIDTH <=  4) ? ((AWIDTH > 12) ? AWIDTH-12 : 0) :
                         (DWIDTH <=  9) ? ((AWIDTH > 11) ? AWIDTH-11 : 0) :
                         (DWIDTH <= 18) ? ((AWIDTH > 10) ? AWIDTH-10 : 0) :
                                          ((AWIDTH >  9) ? AWIDTH-9  : 0);
    localparam  BWIDTHx = (BWIDTH > 0) ? BWIDTH : 1;

    // Number of block RAM cells to use
    localparam  RAMS = (1 << BWIDTH);

    // Actual width we will use
    localparam  XWIDTH = (DWIDTH <=  2) ? DWIDTH :
                         (DWIDTH <=  4) ? 4 :
                         (DWIDTH <=  9) ? 9 :
                         (DWIDTH <= 18) ? 18 :
                                          36;

    // Address shift needed
    localparam  ASHIFT = (DWIDTH ==  1) ? 0 :
                         (DWIDTH ==  2) ? 1 :
                         (DWIDTH <=  4) ? 2 :
                         (DWIDTH <=  9) ? 3 :
                         (DWIDTH <= 18) ? 4 :
                                          5;

    /****************************************/
    /*
     *  Intermediate address.  Left shifted depending on the block RAM data
     *  width used, since the RAM expects the address to be left justified.
     */
    // wire [31:0]         addr0 = (p0_addr << ASHIFT);
    wire [31:0]         addr0 = { {32-AWIDTH-ASHIFT{1'b0}},
                                  p0_addr,
                                  {ASHIFT{1'b0}} };
    wire [31:0]         addr1 = { {32-AWIDTH-ASHIFT{1'b0}},
                                  p1_addr,
                                  {ASHIFT{1'b0}} };

    // Intermediate read data
    wire [31:0]         rdata0;
    wire [3:0]          rparity0;

    wire [31:0]         rdata1;
    wire [3:0]          rparity1;

    // Bank selector
    wire [BWIDTHx-1:0]  bank0;
    wire [BWIDTHx-1:0]  bank1;

    // RAM read and write data
    wire [31:0]         xrdata0[0:RAMS-1];
    wire [3:0]          xrparity0[0:RAMS-1];
    wire [31:0]         xwdata0;
    wire [3:0]          xwparity0;

    wire [31:0]         xrdata1[0:RAMS-1];
    wire [3:0]          xrparity1[0:RAMS-1];
    wire [31:0]         xwdata1;
    wire [3:0]          xwparity1;

    // The remaining RAM signals
    wire [13:0]         xaddr0 = addr0[13:0];
    wire [RAMS-1:0]     xen0 = (p0_en << bank0);
    wire [LANES-1:0]    xbe0 = p0_wen;

    wire [13:0]         xaddr1 = addr1[13:0];
    wire [RAMS-1:0]     xen1 = (p1_en << bank1);
    wire [LANES-1:0]    xbe1 = p1_wen;

    /****************************************/

    genvar i;
    generate

        if (BWIDTH > 0)
        begin
            assign bank0 = addr0[BWIDTH+14-1:14];
            assign rdata0 = xrdata0[bank0];
            assign rparity0 = xrparity0[bank0];

            assign bank1 = addr1[BWIDTH+14-1:14];
            assign rdata1 = xrdata1[bank1];
            assign rparity1 = xrparity1[bank1];
        end else begin
            assign bank0 = 1'b0;
            assign rdata0 = xrdata0[0];
            assign rparity0 = xrparity0[0];

            assign bank1 = 1'b0;
            assign rdata1 = xrdata1[0];
            assign rparity1 = xrparity1[0];
        end

        /*
         *  Route the read and write data correctly, depending on the
         *  data width.
         */
        if (DWIDTH == 9)
        begin
            assign xwdata0 = { 24'b0, p0_wdata[7:0] };
            assign xwparity0 = { 3'b000, p0_wdata[8] };
            assign p0_rdata = { rparity0[0], rdata0[7:0] };

            assign xwdata1 = { 24'b0, p1_wdata[7:0] };
            assign xwparity1 = { 3'b000, p1_wdata[8] };
            assign p1_rdata = { rparity1[0], rdata1[7:0] };
        end
        else if (DWIDTH == 17 || DWIDTH == 18)
        begin
            assign xwdata0 = { 16'b0, p0_wdata[15:0] };
            assign xwparity0 = { {18-DWIDTH+2{1'b0}}, p0_wdata[DWIDTH-1:16] };
            assign p0_rdata = { rparity0[DWIDTH-15-2:0], rdata0[15:0] };

            assign xwdata1 = { 16'b0, p1_wdata[15:0] };
            assign xwparity1 = { {18-DWIDTH+2{1'b0}}, p1_wdata[DWIDTH-1:16] };
            assign p1_rdata = { rparity1[DWIDTH-15-2:0], rdata1[15:0] };
        end
        else if (DWIDTH <= 32)
        begin
            assign xwdata0 = { {32-DWIDTH{1'b0}}, p0_wdata[DWIDTH-1:0] };
            assign xwparity0 = 4'b0000;
            assign p0_rdata = rdata0[DWIDTH-1:0];

            assign xwdata1 = { {32-DWIDTH{1'b0}}, p1_wdata[DWIDTH-1:0] };
            assign xwparity1 = 4'b0000;
            assign p1_rdata = rdata1[DWIDTH-1:0];
        end
        else // if (DWIDTH > 32)
        begin
            assign xwdata0 = p0_wdata[31:0];
            assign xwparity0 = { {DWIDTH-32{1'b0}}, p0_wdata[DWIDTH-1:32] };
            assign p0_rdata = { rparity0[DWIDTH-31-2:0], rdata0[31:0] };

            assign xwdata1 = p1_wdata[31:0];
            assign xwparity1 = { {DWIDTH-32{1'b0}}, p1_wdata[DWIDTH-1:32] };
            assign p1_rdata = { rparity1[DWIDTH-31-2:0], rdata1[31:0] };
        end


        /*
         *  Make enough block RAM cells to cover the requested area.
         */
        for (i = 0; i < RAMS; i = i + 1)
        begin : ram

            RAMB16BWER
            #(
                .DATA_WIDTH_A(XWIDTH),
                .DATA_WIDTH_B(XWIDTH),
                .DOA_REG(0),
                .DOB_REG(0),
                .EN_RSTRAM_A("TRUE"),
                .EN_RSTRAM_B("TRUE"),
                .INIT_FILE("NONE"),
                .RSTTYPE("SYNC"),
                .RST_PRIORITY_A("CE"),
                .RST_PRIORITY_B("CE"),
                .SIM_COLLISION_CHECK("ALL"),
                .SIM_DEVICE("SPARTAN6"),
                .SRVAL_A({XWIDTH{1'b0}}),
                .SRVAL_B({XWIDTH{1'b0}}),
                .WRITE_MODE_A("READ_FIRST"),
                .WRITE_MODE_B("READ_FIRST")
            )
            xram
            (
                .DOA(xrdata0[i]),
                .DOPA(xrparity0[i]),
                .ADDRA(xaddr0),
                .CLKA(clk),
                .ENA(xen0[i]),
                .REGCEA(1'b1),
                .RSTA(rst),
                .WEA({{4-LANES{1'b0}}, xbe0}),
                .DIA(xwdata0),
                .DIPA(xwparity0),

                .DOB(xrdata1[i]),
                .DOPB(xrparity1[i]),
                .ADDRB(xaddr1),
                .CLKB(clk),
                .ENB(xen1[i]),
                .REGCEB(1'b1),
                .RSTB(rst),
                .WEB({{4-LANES{1'b0}}, xbe1}),
                .DIB(xwdata1),
                .DIPB(xwparity1)
            );

        end

    endgenerate

`endif // XILINX

endmodule

`endif // crap2


/**********************************************************************/
/**********************************************************************/

`ifdef crap3

`ifdef XILINX

/*
 *	A wrapper for the Xilinx block RAM.
 */

module XilinxBlockRAM
#(
    parameter	AWIDTH = 9,	        // Address width
                DWIDTH = 22,             // Data width
                LANES = (DWIDTH+7)/8    // (byte lanes)
)
(
    input		    clk,
    input		    rst,

    input [AWIDTH-1 : 0]    p0_addr,
    input		    p0_en,
    input [LANES-1 : 0]     p0_wen,
    input [DWIDTH-1 : 0]    p0_wdata,
    output [DWIDTH-1 : 0]   p0_rdata,

    input [AWIDTH-1 : 0]    p1_addr,
    input		    p1_en,
    input [LANES-1 : 0]     p1_wen,
    input [DWIDTH-1 : 0]    p1_wdata,
    output [DWIDTH-1 : 0]   p1_rdata
);

    /****************************************/

    // The width of the bank field
    localparam  BWIDTH = (DWIDTH ==  1) ? ((AWIDTH > 14) ? AWIDTH-14 : 0) :
                         (DWIDTH ==  2) ? ((AWIDTH > 13) ? AWIDTH-13 : 0) :
                         (DWIDTH <=  4) ? ((AWIDTH > 12) ? AWIDTH-12 : 0) :
                         (DWIDTH <=  9) ? ((AWIDTH > 11) ? AWIDTH-11 : 0) :
                         (DWIDTH <= 18) ? ((AWIDTH > 10) ? AWIDTH-10 : 0) :
                                          ((AWIDTH >  9) ? AWIDTH-9  : 0);
    localparam  BWIDTHx = (BWIDTH > 0) ? BWIDTH : 1;

    // Number of block RAM cells to use
    localparam  RAMS = (1 << BWIDTH);

    // Actual width we will use
    localparam  XWIDTH = (DWIDTH <=  2) ? DWIDTH :
                         (DWIDTH <=  4) ? 4 :
                         (DWIDTH <=  9) ? 9 :
                         (DWIDTH <= 18) ? 18 :
                                          36;

    // Address shift needed
    // localparam  ASHIFT = (DWIDTH ==  1) ? 0 :
                         // (DWIDTH ==  2) ? 1 :
                         // (DWIDTH <=  4) ? 2 :
                         // (DWIDTH <=  9) ? 3 :
                         // (DWIDTH <= 18) ? 4 :
                                          // 5;

    /****************************************/
    /*
     *  Intermediate address.  Left shifted depending on the block RAM data
     *  width used, since the RAM expects the address to be left justified.
     */
    // wire [31:0]         addr0 = (p0_addr << ASHIFT);
    wire [31:0]         addr0 = { {32-AWIDTH{1'b0}}, p0_addr[AWIDTH-1:0] };
    wire [31:0]         addr1 = { {32-AWIDTH{1'b0}}, p1_addr[AWIDTH-1:0] };

    // Intermediate read data
    wire [35:0]         rdata0;
    wire [35:0]         rdata1;

    // Bank selector
    wire [BWIDTHx-1:0]  bank0;
    wire [BWIDTHx-1:0]  bank1;

    // RAM read and write data
    wire [35:0]         xrdata0[0:RAMS-1];
    wire [31:0]         xwdata0;
    wire [3:0]          xwparity0;

    wire [31:0]         xrdata1[0:RAMS-1];
    wire [3:0]          xrparity1[0:RAMS-1];
    wire [31:0]         xwdata1;
    wire [3:0]          xwparity1;

    // The remaining RAM signals
    wire [13:0]         xaddr0 = addr0[13:0];
    wire [RAMS-1:0]     xen0 = (p0_en << bank0);
    wire [LANES-1:0]    xbe0 = p0_wen;

    wire [13:0]         xaddr1 = addr1[13:0];
    wire [RAMS-1:0]     xen1 = (p1_en << bank1);
    wire [LANES-1:0]    xbe1 = p1_wen;

    /****************************************/

    genvar i;
    generate

        if (BWIDTH > 0)
        begin
            assign bank0 = addr0[BWIDTH+14-1:14];
            assign rdata0 = xrdata0[bank0];
            assign rparity0 = xrparity0[bank0];

            assign bank1 = addr1[BWIDTH+14-1:14];
            assign rdata1 = xrdata1[bank1];
            assign rparity1 = xrparity1[bank1];
        end else begin
            assign bank0 = 1'b0;
            assign rdata0 = xrdata0[0];
            assign rparity0 = xrparity0[0];

            assign bank1 = 1'b0;
            assign rdata1 = xrdata1[0];
            assign rparity1 = xrparity1[0];
        end

        /*
         *  Route the read and write data correctly, depending on the
         *  data width.
         */
        if (DWIDTH == 9)
        begin
            assign xwdata0 = { 24'b0, p0_wdata[7:0] };
            assign xwparity0 = { 3'b000, p0_wdata[8] };
            assign p0_rdata = { rparity0[0], rdata0[7:0] };

            assign xwdata1 = { 24'b0, p1_wdata[7:0] };
            assign xwparity1 = { 3'b000, p1_wdata[8] };
            assign p1_rdata = { rparity1[0], rdata1[7:0] };
        end
        else if (DWIDTH == 17 || DWIDTH == 18)
        begin
            assign xwdata0 = { 16'b0, p0_wdata[15:0] };
            assign xwparity0 = { {18-DWIDTH+2{1'b0}}, p0_wdata[DWIDTH-1:16] };
            assign p0_rdata = { rparity0[DWIDTH-15-2:0], rdata0[15:0] };

            assign xwdata1 = { 16'b0, p1_wdata[15:0] };
            assign xwparity1 = { {18-DWIDTH+2{1'b0}}, p1_wdata[DWIDTH-1:16] };
            assign p1_rdata = { rparity1[DWIDTH-15-2:0], rdata1[15:0] };
        end
        else if (DWIDTH <= 32)
        begin
            assign xwdata0 = { {32-DWIDTH{1'b0}}, p0_wdata[DWIDTH-1:0] };
            assign xwparity0 = 4'b0000;
            assign p0_rdata = rdata0[DWIDTH-1:0];

            assign xwdata1 = { {32-DWIDTH{1'b0}}, p1_wdata[DWIDTH-1:0] };
            assign xwparity1 = 4'b0000;
            assign p1_rdata = rdata1[DWIDTH-1:0];
        end
        else // if (DWIDTH > 32)
        begin
            assign xwdata0 = p0_wdata[31:0];
            assign xwparity0 = { {DWIDTH-32{1'b0}}, p0_wdata[DWIDTH-1:32] };
            assign p0_rdata = { rparity0[DWIDTH-31-2:0], rdata0[31:0] };

            assign xwdata1 = p1_wdata[31:0];
            assign xwparity1 = { {DWIDTH-32{1'b0}}, p1_wdata[DWIDTH-1:32] };
            assign p1_rdata = { rparity1[DWIDTH-31-2:0], rdata1[31:0] };
        end


        /*
         *  Make enough block RAM cells to cover the requested area.
         */
        for (i = 0; i < RAMS; i = i + 1)
        begin : ram

            myRAM16
            #(
                .DWIDTH(XWIDTH),
                .ASHIFT(ASHIFT)
            )
            xram
            (
                .DOA(xrdata0[i]),
                .DOPA(xrparity0[i]),
                .ADDRA(xaddr0),
                .CLKA(clk),
                .ENA(xen0[i]),
                .RSTA(rst),
                .WEA({{4-LANES{1'b0}}, xbe0}),
                .DIA(xwdata0),
                .DIPA(xwparity0),

                .DOB(xrdata1[i]),
                .DOPB(xrparity1[i]),
                .ADDRB(xaddr1),
                .CLKB(clk),
                .ENB(xen1[i]),
                .RSTB(rst),
                .WEB({{4-LANES{1'b0}}, xbe1}),
                .DIB(xwdata1),
                .DIPB(xwparity1)
            );

        end

    endgenerate

endmodule


module myRAM16
#(
    parameter	DWIDTH = 8,
                ASHIFT = 0
)
(
    input [31:0]    DOA,
    input [3:0]     DOPA,
    input [13:0]    ADDRA,
    input           CLKA,
    input           ENA,
    input           RSTA,
    input [3:0]     WEA,
    output [31:0]   DIA,
    output [3:0]    DIPA,

    input [31:0]    DOB,
    input [3:0]     DOPB,
    input [13:0]    ADDRB,
    input           CLKB,
    input           ENB,
    input           RSTB,
    input [3:0]     WEB,
    output [31:0]   DIB,
    output [3:0]    DIPB
);

    reg [13:0]      RA, RB;

    always @(posedge CLKA) if (ENA) RA <= ADDRA;
    always @(posedge CLKB) if (ENB) RB <= ADDRB;

    generate

        if (DWIDTH == 1)
        begin
            reg [0:0] mem[0 : 16383];
            assign DIA = { 31'b0, mem[RA[13:0]][0] };
            assign DIB = { 31'b0, mem[RB[13:0]][0] };
            always @(posedge CLKA)
                if (ENA && WEA[0])
                    mem[ADDRA[13:0]] <= DOA[0];
            always @(posedge CLKB)
                if (ENB && WEB[0])
                    mem[ADDRB[13:0]] <= DOB[0];
        end
        else if (DWIDTH == 2)
        begin
            reg [1:0] mem[0 : 8191];
            assign DIA = { 30'b0, mem[RA[13:1]][1:0] };
            assign DIB = { 30'b0, mem[RB[13:1]][1:0] };
            always @(posedge CLKA)
                if (ENA && WEA[0])
                    mem[ADDRA[13:1]] <= DOA[1:0];
            always @(posedge CLKB)
                if (ENB && WEB[0])
                    mem[ADDRB[13:1]] <= DOB[1:0];
        end
        else if (DWIDTH == 4)
        begin
            reg [3:0] mem[0 : 4095];
            assign DIA = { 28'b0, mem[RA[13:2]][3:0] };
            assign DIB = { 28'b0, mem[RB[13:2]][3:0] };
            always @(posedge CLKA)
                if (ENA && WEA[0])
                    mem[ADDRA[13:2]] <= DOA[3:0];
            always @(posedge CLKB)
                if (ENB && WEB[0])
                    mem[ADDRB[13:2]] <= DOB[3:0];
        end
        else if (DWIDTH == 9)
        begin
            reg [8:0] mem[0 : 2047];
            assign DIA = { 23'b0, mem[RA[13:3]][8:0] };
            assign DIB = { 23'b0, mem[RB[13:3]][8:0] };
            always @(posedge CLKA)
                if (ENA && WEA[0])
                    mem[ADDRA[13:3]] <= DOA[8:0];
            always @(posedge CLKB)
                if (ENB && WEB[0])
                    mem[ADDRB[13:3]] <= DOB[8:0];
        end
        else if (DWIDTH == 18)
        begin
            reg [17:0] mem[0 : 1023];
            assign DIA = { 14'b0, mem[RA[13:4]][17:0] };
            assign DIB = { 14'b0, mem[RB[13:4]][17:0] };
            always @(posedge CLKA)
                if (ENA && WEA[0])
                    mem[ADDRA[13:4]][8:0] <= DOA[8:0];
            always @(posedge CLKA)
                if (ENA && WEA[1])
                    mem[ADDRA[13:4]][17:9] <= DOA[17:9];
            always @(posedge CLKB)
                if (ENB && WEB[0])
                    mem[ADDRB[13:4]][8:0] <= DOB[8:0];
            always @(posedge CLKB)
                if (ENB && WEB[1])
                    mem[ADDRB[13:4]][17:9] <= DOB[17:9];
        end
        else if (DWIDTH == 36)
        begin
            reg [35:0] mem[0 : 511];
            assign DIA = { 14'b0, mem[RA[13:4]][17:0] };
            assign DIB = { 14'b0, mem[RB[13:4]][17:0] };
            always @(posedge CLKA)
                if (ENA && WEA[0])
                    mem[ADDRA[13:4]][8:0] <= DOA[8:0];
            always @(posedge CLKA)
                if (ENA && WEA[1])
                    mem[ADDRA[13:4]][17:9] <= DOA[17:9];
            always @(posedge CLKB)
                if (ENB && WEB[0])
                    mem[ADDRB[13:4]][8:0] <= DOB[8:0];
            always @(posedge CLKB)
                if (ENB && WEB[1])
                    mem[ADDRB[13:4]][17:9] <= DOB[17:9];
        end

    endgenerate


`endif // XILINX

`endif // crap3


// vim:set expandtab shiftwidth=4 softtabstop=4 syntax=verilog:
