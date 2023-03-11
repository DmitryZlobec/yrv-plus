`define INTEL_VERSION
`define CLK_FREQUENCY (12 * 1000 * 1000)

`include "yrv_mcu.v"

module top
(
  input              clk,
  input              reset_n,

  input        [3:0] key_sw,
  output       [3:0] led,

  output logic [7:0] abcdefgh,
  output logic [3:0] digit,

  output             buzzer,

  output             hsync,
  output             vsync,
  output       [2:0] rgb,

  output [16:0] sram_a,
  output        sram_n_cs1,
  output        sram_cs2,
  output        sram_n_oe,
  output        sram_n_we,
  inout  [7:0]  sram_io


  `ifdef BOOT_FROM_AUX_UART
  ,
  input              rx
  `endif
);

  //--------------------------------------------------------------------------
  // Unused pins

  assign buzzer = 1'b1;


  //--------------------------------------------------------------------------
  // Slow clock button / switch

  wire slow_clk_mode = ~ key_sw [0];

  //--------------------------------------------------------------------------
  // MCU clock

  logic [22:0] clk_cnt;

  always @ (posedge clk or negedge reset_n)
    if (~ reset_n)
      clk_cnt <= '0;
    else
      clk_cnt <= clk_cnt + 1'd1;

  wire muxed_clk_raw
    = slow_clk_mode ? clk_cnt [22] : clk_cnt[1];


  wire muxed_clk;
  wire video_clk;
  assign video_clk = clk_cnt[0];

  `ifdef SIMULATION
    assign muxed_clk = muxed_clk_raw;
  `else
    global i_global (.in (muxed_clk_raw), .out (muxed_clk));
  `endif

  //--------------------------------------------------------------------------
  // MCU inputs

  wire         ei_req;               // external int request
  wire         nmi_req   = 1'b0;     // non-maskable interrupt
  wire         resetb    = reset_n;  // master reset
  wire         ser_rxd   = 1'b0;     // receive data input
  wire  [15:0] port4_in  = '0;
  wire  [15:0] port5_in  = '0;

  //--------------------------------------------------------------------------
  // MCU outputs

  wire         debug_mode;  // in debug mode
  wire         ser_clk;     // serial clk output (cks mode)
  wire         ser_txd;     // transmit data output
  wire         wfi_state;   // waiting for interrupt
  wire  [15:0] port0_reg;   // port 0
  wire  [15:0] port1_reg;   // port 1
  wire  [15:0] port2_reg;   // port 2
  wire  [15:0] port3_reg;   // port 3

  // Auxiliary UART receive pin

  `ifdef BOOT_FROM_AUX_UART
  wire         aux_uart_rx = rx;
  `endif

  // Exposed memory bus for debug purposes

  wire         mem_ready;   // memory ready
  wire  [31:0] mem_rdata;   // memory read data
  wire         mem_lock;    // memory lock (rmw)
  wire         mem_write;   // memory write enable
  wire   [1:0] mem_trans;   // memory transfer type
  wire   [3:0] mem_ble;     // memory byte lane enables
  wire  [31:0] mem_addr;    // memory address
  wire  [31:0] mem_wdata;   // memory write data

  wire  [31:0] extra_debug_data;


  //Memory bus interface
  reg    [15:0] mem_addr_reg;                              /* reg'd memory address         */
  reg     [3:0] mem_ble_reg;                               /* reg'd memory byte lane en    */


  wire    [3:0] vga_wr_byte_0;                                 /* vga ram byte enables      */
  reg           vga_wr_reg_0;                                  /* mem write                    */

  wire    [3:0] vga_wr_byte_1;                                 /* vga ram byte enables      */
  reg           vga_wr_reg_1;                                  /* mem write                    */


  assign vga_wr_byte_0 = {4{vga_wr_reg_0}} & mem_ble_reg & {4{mem_ready}};
  assign vga_wr_byte_1 = {4{vga_wr_reg_1}} & mem_ble_reg & {4{mem_ready}};


  //--------------------------------------------------------------------------
  // MCU instantiation

  yrv_mcu i_yrv_mcu (.clk (muxed_clk), .*);

  //--------------------------------------------------------------------------
  // Pin assignments

  // The original board had port3_reg [13:8], debug_mode, wfi_state
  assign led = port3_reg [11:8];

  //--------------------------------------------------------------------------

  wire [7:0] abcdefgh_from_mcu =
  {
    port0_reg[6],
    port0_reg[5],
    port0_reg[4],
    port0_reg[3],
    port0_reg[2],
    port0_reg[1],
    port0_reg[0],
    port0_reg[7] 
  };

  wire [3:0] digit_from_mcu =
  {
    port1_reg [3],
    port1_reg [2],
    port1_reg [1],
    port1_reg [0]
  };

  //--------------------------------------------------------------------------

  wire [7:0] abcdefgh_from_show_mode;
  wire [3:0] digit_from_show_mode;

  logic [15:0] display_number;

  always_comb
    casez (key_sw)
    default : display_number = mem_addr  [15: 0];
    4'b110? : display_number = mem_rdata [15: 0];
    4'b100? : display_number = mem_rdata [31:16];
    4'b101? : display_number = mem_wdata [15: 0];
    4'b001? : display_number = mem_wdata [31:16];

    // 4'b101? : display_number = extra_debug_data [15: 0];
    // 4'b001? : display_number = extra_debug_data [31:16];
    endcase

  display_dynamic # (.n_dig (4)) i_display
  (
    .clk       (   clk                     ),
    .reset     ( ~ reset_n                 ),
    .number    (   display_number          ),
    .abcdefgh  (   abcdefgh_from_show_mode ),
    .digit     (   digit_from_show_mode    )
  );

  //--------------------------------------------------------------------------

  always_comb
    if (slow_clk_mode)
    begin
      abcdefgh = abcdefgh_from_show_mode;
      digit    = digit_from_show_mode;
    end
    else
    begin
      abcdefgh = abcdefgh_from_mcu;
      digit    = digit_from_mcu;
    end

  //--------------------------------------------------------------------------

  `ifdef OLD_INTERRUPT_CODE

  //--------------------------------------------------------------------------
  // 125Hz interrupt
  // 50,000,000 Hz / 125 Hz = 40,000 cycles

  logic [15:0] hz125_reg;
  logic        hz125_lat;

  assign ei_req    = hz125_lat;
  wire   hz125_lim = hz125_reg == 16'd39999;

  always_ff @ (posedge clk or negedge resetb)
    if (~ resetb)
    begin
      hz125_reg <= 16'd0;
      hz125_lat <= 1'b0;
    end
    else
    begin
      hz125_reg <= hz125_lim ? 16'd0 : hz125_reg + 1'b1;
      hz125_lat <= ~ port3_reg [15] & (hz125_lim | hz125_lat);
    end

  `endif

  //--------------------------------------------------------------------------
  // 8 KHz interrupt
  // 50,000,000 Hz / 8 KHz = 6250 cycles

  logic [12:0] khz8_reg;
  logic        khz8_lat;

  assign ei_req    = khz8_lat;
  wire   khz8_lim = khz8_reg == 13'd6249;

  always_ff @ (posedge clk or negedge resetb)
    if (~ resetb)
    begin
      khz8_reg <= 13'd0;
      khz8_lat <= 1'b0;
    end
    else
    begin
      khz8_reg <= khz8_lim ? 13'd0 : khz8_reg + 1'b1;
      khz8_lat <= ~ port3_reg [15] & (khz8_lim | khz8_lat);
    end


  localparam X_WIDTH = 10,
             Y_WIDTH = 10,
             CLK_MHZ = 50;

  wire display_on;

    wire [X_WIDTH - 1:0] x;
    wire [Y_WIDTH - 1:0] y;
 
    vga
    # (
        .HPOS_WIDTH ( X_WIDTH      ),
        .VPOS_WIDTH ( Y_WIDTH      ),
        
        .CLK_MHZ    ( CLK_MHZ      )
    )
    i_vga
    (
        .clk        (   clk        ), 
        .reset      ( ~ reset_n    ),
        .hsync      (   hsync      ),
        .vsync      (   vsync      ),
        .display_on (   display_on ),
        .hpos       (   x          ),
        .vpos       (   y          )
    );

    //------------------------------------------------------------------------

    typedef enum bit [2:0]
    {
      black  = 3'b000,
      cyan   = 3'b011,
      red    = 3'b100,
      yellow = 3'b110,
      white  = 3'b111

      // TODO: Add other colors
    }
    rgb_t;



  logic [16:0] pixel_addr;
  wire  [16:0] wr_addr;
  reg   [7:0]  color_reg;// = 8'b00000011;


  // ((y>>1)<<8) + ((y>>1)<<6) + x>>1
  // assign pixel_addr = (((y>>1)*320)+(x>>1));

  always @ (posedge clk) 
  begin
    if(x== 799 && y==524)
      pixel_addr<=0;
    else
      pixel_addr <= (((y>>1)<<8) + ((y>>1)<<6) + ((x)>>1));
  end



  assign sram_cs2 = 1'b1;
  assign sram_n_cs1 = 1'b0;
  assign sram_n_oe = 1'b0;
  assign sram_n_we = ~ we_en;

  wire [7:0] Rx_Data;
  wire  [7:0] Tx_Data;


  assign we_en = (vga_wr_reg_0 || vga_wr_reg_1) && mem_ready;

  assign sram_a  = we_en ? wr_addr : pixel_addr;
  
  assign sram_io = we_en ? Tx_Data: 16'bZ;

  
  always @ (posedge muxed_clk or negedge resetb) begin
    if (!resetb) begin
      mem_addr_reg <= 16'h0;
      mem_ble_reg  <=  4'h0;
      vga_wr_reg_0   <=  1'b0;
      vga_wr_reg_1   <=  1'b0;
    end
    else if (mem_ready) begin
      mem_addr_reg <= mem_addr[15:0];
      mem_ble_reg  <= mem_ble;
      vga_wr_reg_0   <= mem_write && &mem_trans    && (mem_addr[31:16] == `VGA_BASE_0);
      vga_wr_reg_1   <= mem_write && &mem_trans    && (mem_addr[31:16] == `VGA_BASE_1);
      end
  end


  always_comb begin
         if (vga_wr_byte_1[3]) Tx_Data <= mem_wdata[31:24];
    else if (vga_wr_byte_1[2]) Tx_Data <= mem_wdata[23:16];
    else if (vga_wr_byte_1[1]) Tx_Data <= mem_wdata[15:8];
    else if (vga_wr_byte_1[0]) Tx_Data <= mem_wdata[7:0];  
    else if (vga_wr_byte_0[3]) Tx_Data <= mem_wdata[31:24];
    else if (vga_wr_byte_0[2]) Tx_Data <= mem_wdata[23:16];
    else if (vga_wr_byte_0[1]) Tx_Data <= mem_wdata[15:8];
    else                       Tx_Data <= mem_wdata[7:0];  // (vga_wr_byte_0[0]) 
  end

  always_comb begin
         if (vga_wr_byte_0[3]) wr_addr = {1'b0,mem_addr_reg[15:2],1'b1,1'b1};
    else if (vga_wr_byte_0[2]) wr_addr = {1'b0,mem_addr_reg[15:2],1'b1,1'b0};
    else if (vga_wr_byte_0[1]) wr_addr = {1'b0,mem_addr_reg[15:2],1'b0,1'b1};
    else if (vga_wr_byte_0[0]) wr_addr = {1'b0,mem_addr_reg[15:2],1'b0,1'b0};
    else if (vga_wr_byte_1[3]) wr_addr = {1'b1,mem_addr_reg[15:2],1'b1,1'b1};
    else if (vga_wr_byte_1[2]) wr_addr = {1'b1,mem_addr_reg[15:2],1'b1,1'b0};
    else if (vga_wr_byte_1[1]) wr_addr = {1'b1,mem_addr_reg[15:2],1'b0,1'b1};
    else                       wr_addr = {1'b1,mem_addr_reg[15:2],1'b0,1'b0}; //(vga_wr_byte_1[0])
  end




  always @ (posedge clk) begin
    if((! (vga_wr_reg_0 || vga_wr_reg_1))) begin
          color_reg <= sram_io; 
    end  
  end


  always_comb
    begin
      if (~ display_on)
        begin          
          rgb = 3'b000;
        end
      else 
        begin
          // rgb[2] = {color_reg[7] ||color_reg[6] || color_reg[5]};
          // rgb[0] = {color_reg[4] ||color_reg[3] || color_reg[2]};
          // rgb[1] = {color_reg[1] ||color_reg[0]};       

          rgb[2] = {color_reg[2]};
          rgb[1] = {color_reg[1]};
          rgb[0] = {color_reg[0]};   
        end
    end


    // always_comb
    // begin
    //   // Circle

    //   if (~ display_on)
    //     rgb = black;
    //   else if (x ** 2 + y ** 2 < 100 ** 2)
    //     rgb = red;
    //   else if (x > 200 & y > 200 & x < 300 & y < 400) 
    //     rgb = yellow;
    //   else if (key_sw == 4'b1111 & (x - 600) ** 2 + (y - 200) ** 2 < 70 ** 2)
    //     rgb = white;
    //   else
    //     rgb = cyan;

    //   // TODO: Add other figures
    // end

endmodule
