`timescale 1ns / 1ps

/*
 *  A wrapper for block RAM.
 *
 *  Carefully arranged verilog so that the synthesis tools will infer
 *  block RAM of the correct shape.
 */

`define XILINX


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

    /*
     *	Maintain a copy of the read address for when the cache is not actively
     *	reading.  The Xilinx RAM only changes its output when read enable is
     *	active, so we keep supplying the remembered address to the block RAM
     *	so it keeps supplying up to date data.
     */
    reg [AWIDTH-1:0] rdAddr0;
    always @(posedge clock)
        if (readEnable)
            rdAddr0 <= readAddr;
    wire [AWIDTH-1:0] rdAddr = readEnable ? readAddr : rdAddr0;

    /*
     *  Staging variables.
     */
    wire [DWIDTH-1:0] rdData;       // Read data from the RAM cell
    wire bypass0;                   // Same cycle bypass
    reg bypass1;                    // Previous bypass
    reg [DWIDTH-1:0] wrData1;       // Previous write data

    /*
     *	Record if there was a address collision and remember the previous
     *	write data to be supplied on collision.
     */
    always @(posedge clock)
    begin
        bypass1 <= bypass0;         // Remember the collision
        wrData1 <= writeData;       // Remember the alternate data
    end

    /*
     *	True of there is an address collision between the read and the write
     *	sides.	If so, the Xilinx block RAM will either supply old data or
     *	corrupted data, depending on how it was configured.
     */
    assign bypass0 = (rdAddr == writeAddr && writeEnable);

    /*
     *  Route the correct data in response to the read request.
     */
    assign readData = bypass0 ? writeData :
                      bypass1 ? wrData1 :
                                rdData;


    /*
     *  The bypass -- route data from the write side when writing, since the
     *  Xilinx RAM gives unpredictable results when reading from the same
     *  location and a write on the other port.
    wire [DWIDTH-1:0] rdData;
    // wire bypass = (writeAddr == rdAddr && writeEnable);
    reg bypass;
    reg [DWIDTH-1:0] xdata;
    always @(posedge clock)
    begin
        bypass <= (writeAddr == rdAddr && writeEnable);
        xdata <= writeData;
    end
    assign readData = bypass ? writeData : rdData;
     */

    /*
     *  The RAM.
     */
    XilinxBlockRAM
    #(
        .AWIDTH(AWIDTH),
        .DWIDTH(DWIDTH)
    )
    ram
    (
        .clk                (clock),
        .rst                (reset),

        .p0_addr            (rdAddr),       //  (readAddr),
        .p0_en              (1'b1),         //  (readEnable),
        .p0_wen             ({LANES{1'b0}}),
        .p0_wdata           ({DWIDTH{1'b0}}),
        .p0_rdata           (rdData),       //  (readData),

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


/**********************************************************************/
/**********************************************************************/

// `ifdef crap4
`ifdef XILINX

/*
 *	A wrapper for the Xilinx block RAM.
 */

module XilinxBlockRAM
#(
    parameter	AWIDTH = 9,	        // Address width
                DWIDTH = 22,            // Data width
                LANES = 1               // (byte lanes)
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

    //  The RAM cell data width given the address width
    localparam  QW = (AWIDTH <=  9) ? 36 :
                     (AWIDTH == 10) ? 18 :
                     (AWIDTH == 11) ? 9 :
                     (AWIDTH == 12) ? 4 :
                     (AWIDTH == 13) ? 2 :
                     (AWIDTH == 14) ? 1 :
                     0;

    //  Size of the data, parity and byte-enable paths
    localparam  QDW = (QW <= 4) ? QW : (QW & ~7);
    localparam  QPW = (QW <= 4) ?  0 : (QW & 7);
    localparam  QBW = (QW <= 9) ?  1 :
                      (QW <= 18) ? 2 :
                                   4;

    //  Address shift needed to match the Xilinx block RAM
    localparam  AS = (DWIDTH ==  1) ? 0 :
                     (DWIDTH ==  2) ? 1 :
                     (DWIDTH <=  4) ? 2 :
                     (DWIDTH <=  9) ? 3 :
                     (DWIDTH <= 18) ? 4 :
                                      5;

    //  Number of RAM cells needed
    localparam  RAMS = (DWIDTH+QW-1) / QW;

    /****************************************/

    /*
     *	Intermediate address.  Left shifted depending on the block RAM data
     *	width used (QW), since the RAM expects the address to be left
     *	justified.
     */
    wire [31:0]     addr0 = { {32-AWIDTH-AS{1'b0}},
                              p0_addr,
                              {AS{1'b0}} };
    wire [31:0]     addr1 = { {32-AWIDTH-AS{1'b0}},
                              p1_addr,
                              {AS{1'b0}} };

    //  RAM cell read data, one for each RAM call
    wire [QW-1:0]   rData0[0:RAMS];
    wire [QW-1:0]   rData1[0:RAMS];

    //  RAM cell write data
    wire [QW-1:0]   wData0[0:RAMS];
    wire [QW-1:0]   wData1[0:RAMS];

    //  Enable for all RAM cells
    wire            en0 = p0_en;
    wire            en1 = p1_en;

    //  Byte write enable for all RAM cells
    wire [QBW-1:0]  be0 = {QBW{p0_wen}};
    wire [QBW-1:0]  be1 = {QBW{p1_wen}};

    /****************************************/

    function integer o1;
        input [31:0] o0;
        begin
            o1 = o0 + QW - 1;
            if (o1 >= DWIDTH)
                o1 = DWIDTH - 1;
        end
    endfunction

    genvar i, o0;
    generate

        for (o0 = 0; o0 < DWIDTH; o0 = o0 + QW)
        begin
            // Read data routing
            assign p0_rdata[o1(o0):o0] = rData0[o0/QW][o1(o0)-o0:0];
            assign p1_rdata[o1(o0):o0] = rData1[o0/QW][o1(o0)-o0:0];

            // Write data routing
            assign wData0[o0/QW][o1(o0)-o0:0] = p0_wdata[o1(o0):o0];
            assign wData1[o0/QW][o1(o0)-o0:0] = p1_wdata[o1(o0):o0];
        end

        /************************/

        /*
         *  Make enough block RAM cells to cover the requested area.
         */
        for (i = 0; i < RAMS; i = i + 1)
        begin : ram

            RAMB16BWER
            #(
                .DATA_WIDTH_A(QW),
                .DATA_WIDTH_B(QW),
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
                .SRVAL_A({QW{1'b0}}),
                .SRVAL_B({QW{1'b0}}),
                .WRITE_MODE_A("READ_FIRST"),
                .WRITE_MODE_B("READ_FIRST")
            )
            xram
            (
                .DOA(rData0[i][QDW-1:0]),
                .DOPA(rData0[i][QW-1:QDW]),
                .ADDRA(addr0[13:0]),
                .CLKA(clk),
                .ENA(en0),
                .REGCEA(1'b1),
                .RSTA(rst),
                .WEA(be0),
                .DIA(wData0[i][QDW-1:0]),
                .DIPA(wData0[i][QW-1:QDW]),

                .DOB(rData1[i][QDW-1:0]),
                .DOPB(rData1[i][QW-1:QDW]),
                .ADDRB(addr1[13:0]),
                .CLKB(clk),
                .ENB(en1),
                .REGCEB(1'b1),
                .RSTB(rst),
                .WEB(be1),
                .DIB(wData1[i][QDW-1:0]),
                .DIPB(wData1[i][QW-1:QDW])
            );

        end

    endgenerate

endmodule

`endif // XILINX
// `endif // crap4



// vim:set expandtab shiftwidth=4 softtabstop=4 syntax=verilog:
