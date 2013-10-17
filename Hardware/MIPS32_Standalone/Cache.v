/*
 *  The Instruction and Data caches.
 *
 *  The address from the processor is broken up as follows:
 *
 *      31                                s               l     2    0
 *      ---------------+-------------+---------------+---------------+
 *      |               tag               |      set      | off | 00 |
 *      ---------------+-------------+---------------+---------------+
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
 *      Can be 1, 2 or 4.  Lower associativity reduces the total size of
 *      the cache, using less memory, but also reduces the effectivness
 *      of the cache.  Higher associativity results in much better hit
 *      rates at the cost of storage and extra logic.  The best tradeoff
 *      is WAYS=2.  The performance gains from WAYS=1 (direct-mapped) to
 *      WAYS=2 (2-way set associative) is significant;  WAYS > 2 offer
 *      smaller and smaller gains.
 *
 *  Line size (LINE_SIZE):
 *      Can be 4, 8, 16, 32 or 64.  Every memory access requires several
 *      cycles to set up the access (the overhead) and a cycle for each word
 *      accessed.  Smaller line sizes increase the cost of this overhead, but
 *      result in better coverage of memory by the cache, ultimately reducing
 *      miss rates.  Larger line sizes are more efficient with memory access,
 *      but take longer to complete a fill (stalling the CPU for longer).  The
 *      best choice for this implementation is either 16 or 32, depending on
 *      the overhead of the memory controller.
 *
 *  Sets (SETS):
 *      Must be a power-of-2 in size.  This is simply the size of the cache.
 *      Use this to adjust how much block RAM is used.  (Minimum 2, maximum,
 *      very large.)
 *
 *  (Note that at present, this cache does not take into account virtual
 *   vs. physical addresses.  If it is later converted to VIPT cache, a
 *   few changes would be needed, such as a separate refill address register,
 *   since cache addresses and memory addresses would no longer be related.)
 */


/*
 *  TODO:

    *   Handle reset!!!

    *   Verify that RAM address conflicts are dealt with.  So long as the
        read port gives reliable data (either new or old), there is no
        problem.  The Altera block RAM meets this requirement.  The Xilinx
        RAM (at least on the Spartan-6 and Vertex parts), have an issue
        if the RAM is not configured in READ_FIRST mode (the default is
        WRITE_FIRST mode).

    *   Verify that all cofiguration cases are taken care of, particularly
        when WAYS > 1.

    *   How will flush be taken care of?  Flush at reset;  flush by software

    NOTE:

    *   Aborted cached cycles work;  aborting an uncached cycle will give
        wrong data to later cycles.

 */


/*
 *      The Instruction Cache.
 */
module MIPS32_ICache
#(
                                    // Default is a 16K cache
    parameter   WAYS = 2,           // Associativity (1, 2 or 4)
                LINE_SIZE = 16,     // Cache line size (4, 8, 16, 32, 64)
                SETS = 128          // Cache size
)
(
    input               clock,
    input               reset,

    // Processor bus
    input [31:0]        Address,        // Access address
    input               Cached,         // Access should use the cache
    input               Valid,          // Valid access
    output              Stall,          // Result not yet available
    output [31:0]       In,             // Resulting data

    // Wishbone bus
    output [31:0]       ADR_O,
    output              CYC_O,
    output [31:0]       DAT_O,
    output [3:0]        SEL_O,
    output              STB_O,
    output              WE_O,
    output              LOCK_O,
    output [2:0]        CTI_O,
    output [1:0]        BTE_O,
    input               ACK_I,
    input               RTY_I,
    input               ERR_I,
    input [31:0]        DAT_I
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

    localparam  OFF_WIDTH = logb2(LINE_SIZE-1) - 2;
    localparam  OFF_LSB = 2;
    localparam  OFF_MSB = OFF_LSB + OFF_WIDTH - 1;
    localparam  SET_WIDTH = logb2(SETS-1);
    localparam  SET_LSB = OFF_MSB + 1;
    localparam  SET_MSB = SET_LSB + SET_WIDTH - 1;
    localparam  TAG_LSB = SET_MSB + 1;
    localparam  TAG_MSB = 31;
    localparam  TAG_WIDTH = TAG_MSB - TAG_LSB + 1;

    localparam  TA_WIDTH = SET_WIDTH;                   // Tag RAM addr width
    localparam  TD_WIDTH = TAG_WIDTH + 1;               // Tag RAM data width
    localparam  DA_WIDTH = OFF_WIDTH + SET_WIDTH;       // Data RAM addr width

    /****************************************/

    wire [TA_WIDTH-1 : 0]   tagReadAddr;
    wire [TD_WIDTH-1 : 0]   tagReadData[0 : WAYS-1];
    wire [TA_WIDTH-1 : 0]   tagWriteAddr;
    wire [TD_WIDTH-1 : 0]   tagWriteData;
    wire [WAYS-1 : 0]       tagWriteEnable;
    // wire                    tagWriteEnable[0 : WAYS-1];

    wire [DA_WIDTH-1 : 0]   dataReadAddr;
    wire [31 : 0]           dataReadData[0 : WAYS-1];
    wire [DA_WIDTH-1 : 0]   dataWriteAddr;
    wire [31 : 0]           dataWriteData;
    wire [WAYS-1 : 0]       dataWriteEnable;
    // wire                    dataWriteEnable[0 : WAYS-1];

    wire [TAG_WIDTH-1 : 0]  wayTag[0 : WAYS-1];
    wire [WAYS-1:0]         wayValid;
    wire [WAYS-1:0]         wayMatch;
    reg [WAYS-1:0]          waySelect;
    wire                    miss;
    wire [31:0]             cacheData;
    wire                    storeCache;

    wire                    cacheAccess;
    reg                     cacheAccessDelayed;
    wire                    fillStrobe;
    wire                    last;

    wire                    extAccess;
    reg                     extAccessDelayed;
    wire                    extStrobe;

    wire                    ack;

    reg [1:0]               state;      // Refill engine state
    localparam  RS_IDLE = 0;                // No cycle, waiting for processor
    localparam  RS_FILLING = 1;             // Filling a cache line
    localparam  RS_DIRECT = 2;              // Direct uncached cycle
    localparam  RS_INIT = 3;

    reg                     maybeFill;      // We may start a refill next cycle
    reg                     relax;          // Relaxation cycle for wishbone
    wire                    flushing;       // Set if we are flushing

    reg [31:0]              extAddr;        // External (wishbone) address

    /****************************************/
    /*
     *  The cache storage.
     */

    genvar i;
    generate

        for (i = 0; i < WAYS; i = i + 1)
        begin : ram

            /*
             *  Tag RAM.  An entry for each set, wide enough for the tag
             *  plus one valid bit.  One of these RAMs per way.
             */
            MIPS32_RAM
            #(
                .AWIDTH(TA_WIDTH),
                .DWIDTH(TD_WIDTH)
            )
            tagRam
            (
                .clock      (clock),
                .reset      (reset),

                .readAddr   (tagReadAddr),
                .readData   (tagReadData[i]),

                .writeAddr  (tagWriteAddr),
                .writeData  (tagWriteData),
                .writeEnable (tagWriteEnable[i])
            );


            /*
             *  Data RAM.  A 32-bit word for each word in a cache line
             *  times the number of sets.  One of these RAMs for each way.
             */
            MIPS32_RAM
            #(
                .AWIDTH(DA_WIDTH),
                .DWIDTH(32)
            )
            dataRam
            (
                .clock      (clock),
                .reset      (reset),

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
     *  Read from the cache.
     */

    assign tagReadAddr = Address[SET_MSB : SET_LSB];    // Tag RAM address
    assign dataReadAddr = Address[SET_MSB : OFF_LSB];   // Data RAM address

    generate
        // Check for a match on each way of the selected set
        for (i = 0; i < WAYS; i = i + 1)
        begin : match
            assign wayTag[i] = tagReadData[i][TAG_WIDTH : 1];
            assign wayValid[i] = tagReadData[i][0];
            assign wayMatch[i] = (wayTag[i] == Address[TAG_MSB : TAG_LSB] &&
                                  wayValid[i]);
        end

        // Select the data from a valid way
        if (WAYS == 1)
            assign cacheData = dataReadData[0];
        else if (WAYS == 2)
            assign cacheData = wayMatch[0] ? dataReadData[0] :
                                             dataReadData[1];
        else if (WAYS == 4)
            assign cacheData = wayMatch[0] ? dataReadData[0] :
                               wayMatch[1] ? dataReadData[1] :
                               wayMatch[2] ? dataReadData[2] :
                                             dataReadData[3];
    endgenerate

    // Hit or miss?
    assign miss = cacheAccessDelayed && ~(|wayMatch);

    /****************************************/
    /*
     *  Refill logic.
     */

    // True if the current processor cycle is a cached access
    assign cacheAccess = Valid && Cached;

    // True if we are asserting an external cycle for refill
    assign fillStrobe = (maybeFill && miss) || (state == RS_FILLING);

    // True if this is the last refill cycle
    generate
        if (LINE_SIZE == 4)
            assign last = 1'b1;
        else
            assign last = (extAddr[OFF_MSB : OFF_LSB] == {OFF_WIDTH{1'b1}});
    endgenerate

    /****************************************/
    /*
     *  External cycle logic.
     */

     // True if the current processor cycle is an uncached access
    assign extAccess = Valid && !Cached;

    // True if we are asserting an external cycle for a direct access
    assign extStrobe = (state == RS_DIRECT);

    /****************************************/
    /*
     *  Flush, invalidate, evict, ...
     */

    assign flushing = (state == RS_INIT);

    /****************************************/
    /*
     *  Wishbone bus.
     */

    assign CYC_O = fillStrobe || extStrobe;
    assign STB_O = fillStrobe || extStrobe;
    assign CTI_O = (fillStrobe && !last) ? 3'b010 : 3'b111;
    assign BTE_O = 2'b00;
    assign ADR_O = extAddr;
    assign DAT_O = 32'bx;
    assign SEL_O = 4'bx;
    assign WE_O = 1'b0;
    assign LOCK_O = 1'b0;

    // Wishbone ACK (treat retry and error as a regular ack)
    assign ack = ACK_I || RTY_I || ERR_I;

    /****************************************/
    /*
     *  Write to the cache.
     */

    // True if we are to store incoming data to the cache
    assign storeCache = fillStrobe && ack;
    wire storeValid = !flushing;

    // Cache memory addresses to write incoming data
    assign tagWriteAddr = extAddr[SET_MSB : SET_LSB];
    assign tagWriteData = { extAddr[TAG_MSB : TAG_LSB], storeValid };
    assign dataWriteAddr = extAddr[SET_MSB : OFF_LSB];

    // Incoming cache fill data
    assign dataWriteData = DAT_I;

    // Write enable signals for each way of tag and data RAM
    wire tagWriteEnable0 = (storeCache && last) ? 1'b1 : 1'b0;
    wire tagWriteEnable1 = {WAYS{tagWriteEnable0}} & waySelect;
    wire tagWriteEnable2 = {WAYS{flushing}};
    assign tagWriteEnable = tagWriteEnable1 | tagWriteEnable2;

    assign dataWriteEnable0 = (storeCache) ? 1'b1 : 1'b0;
    assign dataWriteEnable = {WAYS{dataWriteEnable0}} & waySelect;

    // Select the correct Way to write the data to
    generate
        // for (i = 0; i < WAYS; i = i + 1)
        // begin : wayWrite
            // assign tagWriteEnable[i] = (storeCache && last) ? waySelect[i] : 0;
            // assign dataWriteEnable[i] = (storeCache) ? waySelect[i] : 0;
        // end

        if (WAYS > 1)
            always @(posedge clock)
                if (reset)
                    waySelect <= {WAYS{1'b1}};
                else if (last && ack)
                    waySelect <= { waySelect[WAYS-2:0], waySelect[WAYS-1] };

    endgenerate

    /****************************************/
    /*
     *  Processor bus.
     */

    // True if the processor should stall, because there's no valid data for it
    assign Stall = (cacheAccessDelayed && miss) ||
                   (extStrobe && !ack) ||
                   relax ||
                   flushing;

    // Route the data to the processor, either from cache or external memory
    assign In = cacheAccessDelayed ? cacheData : DAT_I;

    /**********************************************************************/
    /*
     *	Sequential logic
     */

    always @(posedge clock)
    begin

        if (reset)
        begin

            state <= RS_INIT;
            cacheAccessDelayed <= 1'b0;
            extAccessDelayed <= 1'b0;
            maybeFill <= 1'b0;
            relax <= 1'b0;
            extAddr <= 32'd0;

        end else begin

            // Delay the access type signals
            cacheAccessDelayed <= cacheAccess;
            extAccessDelayed <= extAccess;

            // Clear pulse signals
            maybeFill <= 1'b0;
            relax <= 1'b0;

            // If we retrieved data for the cache, increment the address
            if (storeCache)
                extAddr <= extAddr + 32'd4;

            // The state machine
            case (state)

            RS_IDLE:            // No active external access
            begin
                if (maybeFill && miss && !(last && ack))
                begin
                    /*
                     *	Start the second cycle of the refill.  (Skip the
                     *	additional cycles if our cache has a single word line
                     *	size and the memory responded within a single cycle;
                     *	rare but possible.)
                     */
                    state <= RS_FILLING;
                end
                else if (cacheAccess)
                begin
                    /*
                     *  Provisionally start a refill cycle.  At this point,
                     *  we need to take note of the cached access and potential
                     *  refill, but only if there is a cache miss.
                     */
                    maybeFill <= 1'b1;
                    extAddr <= { Address[31 : SET_LSB], {OFF_WIDTH+2{1'b0}} };
                end
                else if (extAccess)
                begin
                    /*
                     *  Start an external access.
                     */
                    state <= RS_DIRECT;
                    extAddr <= Address;
                end
            end

            RS_FILLING:         // Actively filling the cache
            begin
                if (last && ack)
                begin
                    /*
                     *	We are currently receiving the last word of the fill.
                     *	Stop the refill.
                     */
                    state <= RS_IDLE;
                end
            end

            RS_DIRECT:          // An active external cycle
            begin
                if (ack)
                begin
                    /*
                     *  We received the data from the external memory.
                     *  Exit the external cycle state, and note the need for
                     *  a "relax" cycle next -- we must separate this cycle
                     *  from the next on the Wishbone bus.
                     */
                    state <= RS_IDLE;
                    relax <= 1'b1;
                end
            end

            RS_INIT:
            begin
                if (extAddr[TAG_LSB])
                    state <= RS_IDLE;
                extAddr <= extAddr + (1 << SET_LSB);
            end

            endcase

        end

    end

endmodule


// vim:set expandtab shiftwidth=4 softtabstop=4 syntax=verilog:
