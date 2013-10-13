/*
 *  The Instruction and Data caches.
 *
 *  The virtual address from the processor is broken up as follows:
 *
 *	31                                s               l     2    0
 *	---------------+-------------+---------------+---------------+
 *	|               tag               |      set      | off | 00 |
 *	---------------+-------------+---------------+---------------+
 *
 *  ... where `off' is the offset into a line of the cache, `set' is the
 *  cache line to use from the cache set, and `tag' is stored in the tag
 *  RAM to identify a cache line.  `l' equates to the number of bits in
 *  the offset address, and `s' equates to the number of bits in the
 *  set address plus 4.  (For a 512 set cache, this would be 13.)
 *
 *  When selecting the parameters for either of the caches, the following
 *  rules apply:
 *
 *  Associativity (WAYS):
 *	Can be 1, 2 or 4.  Lower associativity reduces the total size of
 *	the cache, using less memory, but also reduces the effectivness
 *	of the cache.  Higher associativity results in much better hit
 *	rates at the cost of storage and extra logic.  The best tradeoff
 *	is WAYS=2.  The performance gains from WAYS=1 (direct-mapped) to
 *	WAYS=2 (2-way set associative) is significant;  WAYS > 2 offer
 *      smaller and smaller gains.
 *
 *  Line size (LINE_SIZE):
 *	Can be 4, 8, 16, 32 or 64.  Every memory access requires several
 *	cycles to set up the access (the overhead) and a cycle for each word
 *	accessed.  Smaller line sizes increase the cost of this overhead, but
 *	result in better coverage of memory by the cache, ultimately reducing
 *	miss rates.  Larger line sizes are more efficient with memory access,
 *	but take longer to complete a fill (stalling the CPU for longer).  The
 *	best choice for this implementation is either 16 or 32, depending on
 *	the overhead of the memory controller.
 *
 *  Sets (SETS):
 *	Must be a power-of-2 in size.  This is simply the size of the cache.
 *	Use this to adjust how much block RAM is used.
 */


/*
 *	The Instruction Cache.
 */
module MIPS32_ICache
#(
                                    // Default is a 16K cache
    parameter	WAYS = 2,	    // Associativity (1, 2 or 4)
		LINE_SIZE = 16,	    // Cache line size (4, 8, 16, 32, 64)
		SETS = 128,	    // Cache size
)
(
    input		clock,
    input		reset,

    // Processor bus
    input [31:0]	Address,	// Access address
    input		Valid,		// Valid access
    input		Cached,		// Access should use the cache
    output		Stall,		// Result not yet available
    output [31:0]	In,		// Resulting data

    // Wishbone bus
    output [31:0]	ADR_O,
    output		CYC_O,
    output [31:0]	DAT_O,
    output [3:0]	SEL_O,
    output		STB_O,
    output		WE_O,
    output		LOCK_O,
    output [2:0]	CTI_O,
    output [1:0]	BTE_O,
    input		ACK_I,
    input		RTY_I,
    input		ERR_I,
    input [31:0]	DAT_I
);

    /****************************************/

    function integer logb2;
	input [31:0] v;
	begin
	    logb2 = 0;
	    while (v > 0)
	    begin
		logb2 = logb2 + 1;
		v = v >> 1;
	    end
	end
    endfunction

    /****************************************/

    localparam	OFFSET_WIDTH = logb2(LINE_SIZE-1) - 2;
    localparam	OFFSET_LSB = 2;
    localparam	OFFSET_MSB = OFFSET_LSB + OFFSET_WIDTH - 1;
    localparam	SET_WIDTH = logb2(SETS-1);
    localparam	SET_LSB = OFFSET_MSB + 1;
    localparam	SET_MSB = SET_LSB + SET_WIDTH - 1;
    localparam	TAG_LSB = SET_MSB + 1;
    localparam	TAG_MSB = 31;
    localparam	TAG_WIDTH = TAG_MSB - TAG_LSB + 1;

    localparam	TA_WIDTH = SET_WIDTH;			// Tag RAM addr width
    localparam	TD_WIDTH = TAG_WIDTH + 1;		// Tag RAM data width
    localparam	DA_WIDTH = OFFSET_WIDTH + SET_WIDTH;	// Data RAM addr width

    /****************************************/

    wire [TA_WIDTH-1 : 0]   tagReadAddr;
    wire [TD_WIDTH-1 : 0]   tagReadData[0 : WAYS-1];
    wire [TA_WIDTH-1 : 0]   tagWriteAddr;
    wire [TD_WIDTH-1 : 0]   tagWriteData;
    wire		    tagWriteEnable[0 : WAYS-1];

    wire [DA_WIDTH-1 : 0]   dataReadAddr;
    wire [31 : 0]	    dataReadData[0 : WAYS-1];
    wire [DA_WIDTH-1 : 0]   dataWriteAddr;
    wire [31 : 0]	    dataWriteData;
    wire		    dataWriteEnable[0 : WAYS-1];

    wire [TAG_WIDTH-1 : 0]  wayTag[0 : WAYS-1];
    wire		    wayValid[0 : WAYS-1];
    wire		    wayMatch[0 : WAYS-1];

    wire		    miss;
    wire [31:0]             data;

    reg [1:0]               state;      // Refill engine state
    localparam  RS_IDLE = 0;                // No cycle, waiting for processor
    localparam  RS_REFILL = 1;              // Filling a cache line
    // localparam  RS_FLUSH = 2;
    localparam  RS_WISHBONE = 3;            // Direct uncached cycle

    reg [DA_WIDTH-1:0]      refillAddr;

    // reg [OFFSET_WIDTH-1:0]  refillCount;

    /****************************************/
    /*
     *	The cache storage.
     */

    genvar i;
    generate

	for (i = 0; i < WAYS; i = i + 1)
	begin : ram

	    /*
	     *	Tag RAM.  An entry for each set, wide enough for the tag
	     *	plus one valid bit.  One of these RAMs per way.
	     */
	    MIPS32_RAM
	    #(
		.AWIDTH(TA_WIDTH),
		.DWIDTH(TD_WIDTH)
	    )
	    tagRam
	    (
		.clock	    (clock),
		.reset	    (reset),

		.readAddr   (tagReadAddr),
		.readData   (tagReadData[i]),

		.writeAddr  (tagWriteAddr),
		.writeData  (tagWriteData),
		.writeEnable (tagWriteEnable[i])
	    );


	    /*
	     *	Data RAM.  A 32-bit word for each word in a cache line
	     *	times the number of sets.  One of these RAMs for each way.
	     */
	    MIPS32_RAM
	    #(
		.AWIDTH(DA_WIDTH),
		.DWIDTH(32)
	    )
	    dataRam
	    (
		.clock	    (clock),
		.reset	    (reset),

		.readAddr   (dataReadAddr),
		.readData   (dataReadData[i]),

		.writeAddr  (dataWriteAddr),
		.writeData  (dataWriteData),
		.writeEnable (dataWriteEnable[i])
	    );

	end

    endgenerate

    /****************************************/
    /*
     *	Combinatorial logic.
     */

    // Calculate the address of the requested cache word for the tag & data
    assign tagReadAddr = Address[SET_MSB : SET_LSB];
    assign dataReadAddr = Address[SET_MSB : OFFSET_LSB];

    // Check for a match on each way of the selected set
    generate
	for (i = 0; i < WAYS; i = i + 1)
	    assign wayMatch[i] = wayTag[i] == Address[TAG_MSB : TAG_LSB] &&
				 wayValid[i];
    endgenerate

    // Hit or miss?
    assign miss = ~|wayMatch;

    // Get data from the cache for the CPU
    generate
	if (WAYS == 1)
	    assign data = dataReadData[0];
	else if (WAYS == 2)
	    assign data = wayMatch[0] ? dataReadData[0] :
			                dataReadData[1];
	else if (WAYS == 4)
	    assign data = wayMatch[0] ? dataReadData[0] :
                          wayMatch[1] ? dataReadData[1] :
                          wayMatch[2] ? dataReadData[2] :
                                        dataReadData[3];
    endgenerate

    // True if we should refill
    assign refill = (state == RS_REFILL);
    // assign refill0 = Valid && miss;
    // assign refill1 = (state == RS_REFILL);
    // assign refill = refill0 || refill1;

    // The refill address -- the address at the start of the cache line
    assign refillAddr0 = { Address[31 : SET_LSB], { OFFSET_WIDTH+2{1'b0}} };

    // True if this is the last refill cycle
    generate
        if (LINE_SIZE == 4)
            assign last = 1'b1;
        else
            assign last = refillAddr[OFFSET_WIDTH-1:0] == {OFFSET_WIDTH{1'b1}};
            // assign last = (refillCount == 0);
    endgenerate

    // True if we have an uncached cycle
    assign uncached = Valid && !Cached;

    // Wishbone ACK (treat retry and error as a regular ack)
    assign ack = ACK_I || RTY_I || ERR_I;

    /****************************************/
    /*
     *	Wishbone bus.
     */
    // assign CYC_O = !reset && (refill || uncached);
    // assign STB_O = CYC_O;
    // assign CTI_O = (refill && !last) ? 3'b010 : 3'b111;
    // assign BTE_O = 2'b00;
    // assign ADR_O = !refill ? Address :
    //                refill0 ? refillAddr0 :
    //                          refillAddr1;
    // assign ADR_O = refill ? refillAddr : Address;
    // assign DAT_O = 32'bx;
    // assign SEL_O = 4'bx;
    // assign WE_O = 1'b0;
    // assign LOCK_O = 1'b0;


    assign CYC_O = (state == RS_REFILL || state == RS_WISHBONE);
    assign STB_O = CYC_O;
    assign CTI_O = (refill && !last) ? 3'b010 : 3'b111;
    assign BTE_O = 2'b00;
    assign ADR_O = wbAddr;
    assign DAT_O = 32'bx;
    assign SEL_O = 4'bx;
    assign WE_O = 1'b0;
    assign LOCK_O = 1'b0;

    /*
     *	Processor bus.
     */
    assign Stall = miss || (uncached && !ack);
    assign In = Cached ? data : DAT_I;

    /****************************************/
    /*
     *	Write to the cache.
     */

    assign tagWriteAddr = wbAddr[SET_MSB : SET_LSB];
    assign tagWriteData = { wbAddr[TAG_MSB : TAG_LSB], 1'b1 };
    assign dataWriteAddr = wbAddr[SET_MSB : OFFSET_LSB];
    assign dataWriteData = DAT_I;

    generate
	for (i = 0; i < WAYS; i = i + 1)
	begin : wayWrite
            assign tagWriteEnable[i] = (waySelect == i) && last;
            assign dataWriteEnable[i] = (waySelect == i);
        end
    endgenerate

    /****************************************/
    /*
     *	State machine.
     */

    always @(posedge clock)
    begin

        case (state)
            RS_IDLE:
            begin
                if (Valid && Cached && miss)
                begin
                    state <= RS_REFILL;
                    wbAddr <= { Address[31 : SET_LSB], {OFFSET_WIDTH+2{1'b0}} };
                    refillCount <= {OFFSET_WIDTH{1'b1}};
                end

                if (Valid && !Cached)
                begin
                    state <= RS_WISHBONE;
                    wbAddr <= Address;
                end
            end

            RS_REFILL:
            begin
                if (ack)
                begin
                    wbAddr <= wbAddr + 32'd4;
                    refillCount <= refillCount - 1;

                    if (last)
                    begin
                        state <= RS_IDLE;

                        if (waySelect == WAYS-1)
                            waySelect <= 0;
                        else
                            waySelect <= waySelect + 1;
                    end
                end
            end

            RS_WISHBONE:
            begin
                if (ack)
                    state <= RS_IDLE;
            end
        endcase


    end

endmodule

// vim:set expandtab shiftwidth=4 softtabstop=4 syntax=verilog:
