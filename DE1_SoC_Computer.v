

module DE1_SoC_Computer (
	////////////////////////////////////
	// FPGA Pins
	////////////////////////////////////

	// Clock pins
	CLOCK_50,
	CLOCK2_50,
	CLOCK3_50,
	CLOCK4_50,

	// ADC
	ADC_CS_N,
	ADC_DIN,
	ADC_DOUT,
	ADC_SCLK,

	// Audio
	AUD_ADCDAT,
	AUD_ADCLRCK,
	AUD_BCLK,
	AUD_DACDAT,
	AUD_DACLRCK,
	AUD_XCK,

	// SDRAM
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_LDQM,
	DRAM_RAS_N,
	DRAM_UDQM,
	DRAM_WE_N,

	// I2C Bus for Configuration of the Audio and Video-In Chips
	FPGA_I2C_SCLK,
	FPGA_I2C_SDAT,

	// 40-Pin Headers
	GPIO_0,
	GPIO_1,
	
	// Seven Segment Displays
	HEX0,
	HEX1,
	HEX2,
	HEX3,
	HEX4,
	HEX5,

	// IR
	IRDA_RXD,
	IRDA_TXD,

	// Pushbuttons
	KEY,

	// LEDs
	LEDR,

	// PS2 Ports
	PS2_CLK,
	PS2_DAT,
	
	PS2_CLK2,
	PS2_DAT2,

	// Slider Switches
	SW,

	// Video-In
	TD_CLK27,
	TD_DATA,
	TD_HS,
	TD_RESET_N,
	TD_VS,

	// VGA
	VGA_B,
	VGA_BLANK_N,
	VGA_CLK,
	VGA_G,
	VGA_HS,
	VGA_R,
	VGA_SYNC_N,
	VGA_VS,

	////////////////////////////////////
	// HPS Pins
	////////////////////////////////////
	
	// DDR3 SDRAM
	HPS_DDR3_ADDR,
	HPS_DDR3_BA,
	HPS_DDR3_CAS_N,
	HPS_DDR3_CKE,
	HPS_DDR3_CK_N,
	HPS_DDR3_CK_P,
	HPS_DDR3_CS_N,
	HPS_DDR3_DM,
	HPS_DDR3_DQ,
	HPS_DDR3_DQS_N,
	HPS_DDR3_DQS_P,
	HPS_DDR3_ODT,
	HPS_DDR3_RAS_N,
	HPS_DDR3_RESET_N,
	HPS_DDR3_RZQ,
	HPS_DDR3_WE_N,

	// Ethernet
	HPS_ENET_GTX_CLK,
	HPS_ENET_INT_N,
	HPS_ENET_MDC,
	HPS_ENET_MDIO,
	HPS_ENET_RX_CLK,
	HPS_ENET_RX_DATA,
	HPS_ENET_RX_DV,
	HPS_ENET_TX_DATA,
	HPS_ENET_TX_EN,

	// Flash
	HPS_FLASH_DATA,
	HPS_FLASH_DCLK,
	HPS_FLASH_NCSO,

	// Accelerometer
	HPS_GSENSOR_INT,
		
	// General Purpose I/O
	HPS_GPIO,
		
	// I2C
	HPS_I2C_CONTROL,
	HPS_I2C1_SCLK,
	HPS_I2C1_SDAT,
	HPS_I2C2_SCLK,
	HPS_I2C2_SDAT,

	// Pushbutton
	HPS_KEY,

	// LED
	HPS_LED,
		
	// SD Card
	HPS_SD_CLK,
	HPS_SD_CMD,
	HPS_SD_DATA,

	// SPI
	HPS_SPIM_CLK,
	HPS_SPIM_MISO,
	HPS_SPIM_MOSI,
	HPS_SPIM_SS,

	// UART
	HPS_UART_RX,
	HPS_UART_TX,

	// USB
	HPS_CONV_USB_N,
	HPS_USB_CLKOUT,
	HPS_USB_DATA,
	HPS_USB_DIR,
	HPS_USB_NXT,
	HPS_USB_STP
);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

////////////////////////////////////
// FPGA Pins
////////////////////////////////////

// Clock pins
input						CLOCK_50;
input						CLOCK2_50;
input						CLOCK3_50;
input						CLOCK4_50;

// ADC
inout						ADC_CS_N;
output					ADC_DIN;
input						ADC_DOUT;
output					ADC_SCLK;

// Audio
input						AUD_ADCDAT;
inout						AUD_ADCLRCK;
inout						AUD_BCLK;
output					AUD_DACDAT;
inout						AUD_DACLRCK;
output					AUD_XCK;

// SDRAM
output 		[12: 0]	DRAM_ADDR;
output		[ 1: 0]	DRAM_BA;
output					DRAM_CAS_N;
output					DRAM_CKE;
output					DRAM_CLK;
output					DRAM_CS_N;
inout			[15: 0]	DRAM_DQ;
output					DRAM_LDQM;
output					DRAM_RAS_N;
output					DRAM_UDQM;
output					DRAM_WE_N;

// I2C Bus for Configuration of the Audio and Video-In Chips
output					FPGA_I2C_SCLK;
inout						FPGA_I2C_SDAT;

// 40-pin headers
inout			[35: 0]	GPIO_0;
inout			[35: 0]	GPIO_1;

// Seven Segment Displays
output		[ 6: 0]	HEX0;
output		[ 6: 0]	HEX1;
output		[ 6: 0]	HEX2;
output		[ 6: 0]	HEX3;
output		[ 6: 0]	HEX4;
output		[ 6: 0]	HEX5;

// IR
input						IRDA_RXD;
output					IRDA_TXD;

// Pushbuttons
input			[ 3: 0]	KEY;

// LEDs
output		[ 9: 0]	LEDR;

// PS2 Ports
inout						PS2_CLK;
inout						PS2_DAT;

inout						PS2_CLK2;
inout						PS2_DAT2;

// Slider Switches
input			[ 9: 0]	SW;

// Video-In
input						TD_CLK27;
input			[ 7: 0]	TD_DATA;
input						TD_HS;
output					TD_RESET_N;
input						TD_VS;

// VGA
output		[ 7: 0]	VGA_B;
output					VGA_BLANK_N;
output					VGA_CLK;
output		[ 7: 0]	VGA_G;
output					VGA_HS;
output		[ 7: 0]	VGA_R;
output					VGA_SYNC_N;
output					VGA_VS;



////////////////////////////////////
// HPS Pins
////////////////////////////////////
	
// DDR3 SDRAM
output		[14: 0]	HPS_DDR3_ADDR;
output		[ 2: 0]  HPS_DDR3_BA;
output					HPS_DDR3_CAS_N;
output					HPS_DDR3_CKE;
output					HPS_DDR3_CK_N;
output					HPS_DDR3_CK_P;
output					HPS_DDR3_CS_N;
output		[ 3: 0]	HPS_DDR3_DM;
inout			[31: 0]	HPS_DDR3_DQ;
inout			[ 3: 0]	HPS_DDR3_DQS_N;
inout			[ 3: 0]	HPS_DDR3_DQS_P;
output					HPS_DDR3_ODT;
output					HPS_DDR3_RAS_N;
output					HPS_DDR3_RESET_N;
input						HPS_DDR3_RZQ;
output					HPS_DDR3_WE_N;

// Ethernet
output					HPS_ENET_GTX_CLK;
inout						HPS_ENET_INT_N;
output					HPS_ENET_MDC;
inout						HPS_ENET_MDIO;
input						HPS_ENET_RX_CLK;
input			[ 3: 0]	HPS_ENET_RX_DATA;
input						HPS_ENET_RX_DV;
output		[ 3: 0]	HPS_ENET_TX_DATA;
output					HPS_ENET_TX_EN;

// Flash
inout			[ 3: 0]	HPS_FLASH_DATA;
output					HPS_FLASH_DCLK;
output					HPS_FLASH_NCSO;

// Accelerometer
inout						HPS_GSENSOR_INT;

// General Purpose I/O
inout			[ 1: 0]	HPS_GPIO;

// I2C
inout						HPS_I2C_CONTROL;
inout						HPS_I2C1_SCLK;
inout						HPS_I2C1_SDAT;
inout						HPS_I2C2_SCLK;
inout						HPS_I2C2_SDAT;

// Pushbutton
inout						HPS_KEY;

// LED
inout						HPS_LED;

// SD Card
output					HPS_SD_CLK;
inout						HPS_SD_CMD;
inout			[ 3: 0]	HPS_SD_DATA;

// SPI
output					HPS_SPIM_CLK;
input						HPS_SPIM_MISO;
output					HPS_SPIM_MOSI;
inout						HPS_SPIM_SS;

// UART
input						HPS_UART_RX;
output					HPS_UART_TX;

// USB
inout						HPS_CONV_USB_N;
input						HPS_USB_CLKOUT;
inout			[ 7: 0]	HPS_USB_DATA;
input						HPS_USB_DIR;
input						HPS_USB_NXT;
output					HPS_USB_STP;

//=======================================================
//  REG/WIRE declarations
//=======================================================

wire			[15: 0]	hex3_hex0;
//wire			[15: 0]	hex5_hex4;

//assign HEX0 = ~hex3_hex0[ 6: 0]; // hex3_hex0[ 6: 0]; 
//assign HEX1 = ~hex3_hex0[14: 8];
//assign HEX2 = ~hex3_hex0[22:16];
//assign HEX3 = ~hex3_hex0[30:24];
assign HEX4 = 7'b1111111;
assign HEX5 = 7'b1111111;

HexDigit Digit0(HEX0, hex3_hex0[3:0]);
HexDigit Digit1(HEX1, hex3_hex0[7:4]);
HexDigit Digit2(HEX2, hex3_hex0[11:8]);
HexDigit Digit3(HEX3, hex3_hex0[15:12]);



//=======================================================
// Bus controller for AVALON bus-master
//=======================================================
// computes DDS for sine wave and fills audio FIFO

reg [31:0] bus_addr ; // Avalon address
// see 
// ftp://ftp.altera.com/up/pub/Altera_Material/15.1/University_Program_IP_Cores/Audio_Video/Audio.pdf
// for addresses
wire [31:0] audio_base_address = 32'h00003040 ;  // Avalon address
wire [31:0] audio_fifo_address = 32'h00003044 ;  // Avalon address +4 offset
wire [31:0] audio_left_address = 32'h00003048 ;  // Avalon address +8
wire [31:0] audio_right_address = 32'h0000304c ;  // Avalon address +12
reg [3:0] bus_byte_enable ; // four bit byte read/write mask
reg bus_read  ;       // high when requesting data
reg bus_write ;      //  high when writing data
reg [31:0] bus_write_data ; //  data to send to Avalog bus
wire bus_ack  ;       //  Avalon bus raises this when done
wire [31:0] bus_read_data ; // data from Avalon bus
reg [30:0] timer ;
reg [3:0] state ;
wire state_clock ;

// current free words in audio interface
reg [7:0] fifo_space ;
// debug check of space
//assign LEDR = fifo_space ;

// use 4-byte-wide bus-master	 
//assign bus_byte_enable = 4'b1111;

// DDS signals
reg [31:0] dds_accum ;
// DDS LUT
//wire [15:0] sine_out ;
wire signed[15:0] sine_out ;
wire signed [26:0] audio_left;
wire signed [26:0] audio_right;
wire donel;
wire doner;
wire donef;


wire signed [26:0] flute_out;
wire signed [26:0] violin_out;
wire signed [26:0] bass_out;
wire signed [26:0] left_in;
wire signed [26:0] right_in;
wire signed [26:0] front_in;
wire signed [19:0] testbench_out1;
wire [7:0] test1_wire;
wire [10:0] test2_wire;


wire signed [26:0] audio_left_test;
wire signed [26:0] audio_right_test;

wire signed [26:0] left_audio_left;
wire signed [26:0] right_audio_left;
wire signed [26:0] left_audio_right;
wire signed [26:0] right_audio_right;
wire signed [26:0] front_audio_left;
wire signed [26:0] front_audio_right;


assign audio_left = left_audio_left + right_audio_left+ front_audio_left;
assign audio_right = left_audio_right + right_audio_right + front_audio_right;


//assign done = 1'b1;
wire fir_done;
reg audio_done = 1'b1;
//assign fir_done = audio_done || done;
assign fir_done = donel && doner && donef;
score DUT(.clk(CLOCK_50), .reset(reset), .output_wire(flute_out), .output_wire2(violin_out), .output_wire3(bass_out), .violin2_counter(testbench_out1), .m10k_wire(test1_wire), .counter_wire(test2_wire), .fir_done(fir_done));
FIR_filter_l filtl(.x(left_in), .clk(CLOCK_50), .reset(reset), .yn_left(left_audio_left), .yn_right(left_audio_right), .done(donel), .audio_done(audio_done));
FIR_filter_r filtr(.x(right_in), .clk(CLOCK_50), .reset(reset), .yn_left(right_audio_left), .yn_right(right_audio_right), .done(doner), .audio_done(audio_done));
FIR_filter_f filtf(.x(front_in), .clk(CLOCK_50), .reset(reset), .yn_left(front_audio_left), .yn_right(front_audio_right), .done(donef), .audio_done(audio_done));

assign left_in = (direction == 2'b11) ? (violin_out<<<7) : ((direction == 2'b10) ? (flute_out<<<7) : (bass_out<<<7));
assign right_in = (direction == 2'b11) ? (flute_out<<<7) : ((direction == 2'b10) ? (bass_out<<<7) : (violin_out<<<7));
assign front_in = (direction == 2'b11) ? (bass_out<<<7) :  ((direction == 2'b10) ? (violin_out<<<7) : (flute_out<<<7));


// get some signals exposed
// connect bus master signals to i/o for probes
assign GPIO_0[0] = bus_write ;
assign GPIO_0[1] = bus_read ;
assign GPIO_0[2] = bus_ack ;
//assign GPIO_0[3] = ??? ;

wire [31:0] pio_reset;
wire reset;
wire [1:0] direction;
assign reset = pio_reset[0];
assign direction = pio_reset[2:1];

always @(posedge CLOCK_50) begin //CLOCK_50

	// reset state machine and read/write controls
	if (reset) begin
		state <= 0 ;
		bus_read <= 0 ; // set to one if a read opeation from bus
		bus_write <= 0 ; // set to one if a write operation to bus
		timer <= 0;
	end
	else begin
		// timer just for deubgging
		timer <= timer + 1;
	end
	
	
	
	// set up read FIFO available space
	if (state==4'd0) begin
		bus_addr <= audio_fifo_address ;
		bus_read <= 1'b1 ;
		bus_byte_enable <= 4'b1111;
		state <= 4'd1 ; // wait for read ACK
	end
	
	// wait for read ACK and read the fifo available
	// bus ACK is high when data is available
	if (state==4'd1 && bus_ack==1) begin
		state <= 4'd2 ; //4'd2
		// FIFO space is in high byte
		fifo_space <= (bus_read_data>>24) ;
		// end the read
		bus_read <= 1'b0 ;
	end
	
	// When there is room in the FIFO
	// -- compute next DDS sine sample
	// -- start write to fifo for each channel
	// -- first the left channel
	if (state==4'd2 && fifo_space>8'd2 ) begin // 
		audio_done <= 1'b1;
		if(fir_done) begin
		state <= 4'd3;	
		// IF SW=10'h200 
		// and Fout = (sample_rate)/(2^32)*{SW[9:0], 16'b0}
		// then Fout=48000/(2^32)*(2^25) = 375 Hz
		dds_accum <= dds_accum + {SW[9:0], 16'b0} ;
		// convert 16-bit table to 32-bit format
		bus_write_data <= (audio_left << 5);
		//else if(reset==2'd2) begin bus_write_data <= (sine_out << 16) ; dds_accum <= dds_accum + {SW[9:0], 16'b0} ; end
		bus_addr <= audio_left_address ;
		bus_byte_enable <= 4'b1111;
		bus_write <= 1'b1 ;
		end
		
	end	
	// if no space, try again later
	else if (state==4'd2 && fifo_space<=8'd2) begin
		state <= 4'b0 ;
		audio_done <= 1'b0;
	end
	
	// detect bus-transaction-complete ACK 
	// for left channel write
	// You MUST do this check
	if (state==4'd3 && bus_ack==1) begin
		state <= 4'd4 ;
		bus_write <= 0;
		audio_done <= 1'b1;
	end
	
	// -- now the right channel
	if (state==4'd4) begin // 
		state <= 4'd5;	
		bus_write_data <= (audio_right << 5);
		//else if(reset ==2'd2) begin bus_write_data <= (sine_out << 16) ; end
		bus_addr <= audio_right_address ;
		bus_write <= 1'b1 ;
	end	
	
	// detect bus-transaction-complete ACK
	// for right channel write
	// You MUST do this check
	if (state==4'd5 && bus_ack==1) begin
		state <= 4'd0 ;
		bus_write <= 0;
	end
	
	
end // always @(posedge state_clock)






//=======================================================
//  Structural coding
//=======================================================

Computer_System The_System (
	////////////////////////////////////
	// FPGA Side
	////////////////////////////////////

	// Global signals
	.system_pll_ref_clk_clk					(CLOCK_50),
	.system_pll_ref_reset_reset			(1'b0),

	// AV Config
	.av_config_SCLK							(FPGA_I2C_SCLK),
	.av_config_SDAT							(FPGA_I2C_SDAT),

	// Audio Subsystem
	.audio_pll_ref_clk_clk					(CLOCK3_50),
	.audio_pll_ref_reset_reset				(1'b0),
	.audio_clk_clk								(AUD_XCK),
	.audio_ADCDAT								(AUD_ADCDAT),
	.audio_ADCLRCK								(AUD_ADCLRCK),
	.audio_BCLK									(AUD_BCLK),
	.audio_DACDAT								(AUD_DACDAT),
	.audio_DACLRCK								(AUD_DACLRCK),

	// Slider Switches
	.slider_switches_export					(SW),

	// Pushbuttons (~KEY[3:0]),
	.pushbuttons_export						(~KEY[3:0]),

	// Expansion JP1
	//.expansion_jp1_export					({GPIO_0[35:19], GPIO_0[17], GPIO_0[15:3], GPIO_0[1]}),

	// Expansion JP2
	//.expansion_jp2_export					({GPIO_1[35:19], GPIO_1[17], GPIO_1[15:3], GPIO_1[1]}),

	// LEDs
	.leds_export								(LEDR),
	
	// Seven Segs
	.hex3_hex0_export							(hex3_hex0),
	//.hex5_hex4_export							(hex5_hex4),
	
	// PS2 Ports
	//.ps2_port_CLK								(PS2_CLK),
	//.ps2_port_DAT								(PS2_DAT),
	//.ps2_port_dual_CLK						(PS2_CLK2),
	//.ps2_port_dual_DAT						(PS2_DAT2),

	// IrDA
	//.irda_RXD									(IRDA_RXD),
	//.irda_TXD									(IRDA_TXD),

	// VGA Subsystem
	.vga_pll_ref_clk_clk 					(CLOCK2_50),
	.vga_pll_ref_reset_reset				(1'b0),
	.vga_CLK										(VGA_CLK),
	.vga_BLANK									(VGA_BLANK_N),
	.vga_SYNC									(VGA_SYNC_N),
	.vga_HS										(VGA_HS),
	.vga_VS										(VGA_VS),
	.vga_R										(VGA_R),
	.vga_G										(VGA_G),
	.vga_B										(VGA_B),
	
	// Video In Subsystem
//	.video_in_TD_CLK27 						(TD_CLK27),
//	.video_in_TD_DATA							(TD_DATA),
//	.video_in_TD_HS							(TD_HS),
//	.video_in_TD_VS							(TD_VS),
//	.video_in_clk27_reset					(),
//	.video_in_TD_RESET						(TD_RESET_N),
//	.video_in_overflow_flag					(),
	
	// SDRAM
	.sdram_clk_clk								(DRAM_CLK),
   .sdram_addr									(DRAM_ADDR),
	.sdram_ba									(DRAM_BA),
	.sdram_cas_n								(DRAM_CAS_N),
	.sdram_cke									(DRAM_CKE),
	.sdram_cs_n									(DRAM_CS_N),
	.sdram_dq									(DRAM_DQ),
	.sdram_dqm									({DRAM_UDQM,DRAM_LDQM}),
	.sdram_ras_n								(DRAM_RAS_N),
	.sdram_we_n									(DRAM_WE_N),
	
	
	// bus-master state machine interface
	.bus_master_audio_external_interface_address     (bus_addr),     
	.bus_master_audio_external_interface_byte_enable (bus_byte_enable), 
	.bus_master_audio_external_interface_read        (bus_read),        
	.bus_master_audio_external_interface_write       (bus_write),       
	.bus_master_audio_external_interface_write_data  (bus_write_data),  
	.bus_master_audio_external_interface_acknowledge (bus_ack),                                  
	.bus_master_audio_external_interface_read_data   (bus_read_data), 
	
	////////////////////////////////////
	// HPS Side
	////////////////////////////////////
	// DDR3 SDRAM
	.memory_mem_a			(HPS_DDR3_ADDR),
	.memory_mem_ba			(HPS_DDR3_BA),
	.memory_mem_ck			(HPS_DDR3_CK_P),
	.memory_mem_ck_n		(HPS_DDR3_CK_N),
	.memory_mem_cke		(HPS_DDR3_CKE),
	.memory_mem_cs_n		(HPS_DDR3_CS_N),
	.memory_mem_ras_n		(HPS_DDR3_RAS_N),
	.memory_mem_cas_n		(HPS_DDR3_CAS_N),
	.memory_mem_we_n		(HPS_DDR3_WE_N),
	.memory_mem_reset_n	(HPS_DDR3_RESET_N),
	.memory_mem_dq			(HPS_DDR3_DQ),
	.memory_mem_dqs		(HPS_DDR3_DQS_P),
	.memory_mem_dqs_n		(HPS_DDR3_DQS_N),
	.memory_mem_odt		(HPS_DDR3_ODT),
	.memory_mem_dm			(HPS_DDR3_DM),
	.memory_oct_rzqin		(HPS_DDR3_RZQ),
		  
	// Ethernet
	.hps_io_hps_io_gpio_inst_GPIO35	(HPS_ENET_INT_N),
	.hps_io_hps_io_emac1_inst_TX_CLK	(HPS_ENET_GTX_CLK),
	.hps_io_hps_io_emac1_inst_TXD0	(HPS_ENET_TX_DATA[0]),
	.hps_io_hps_io_emac1_inst_TXD1	(HPS_ENET_TX_DATA[1]),
	.hps_io_hps_io_emac1_inst_TXD2	(HPS_ENET_TX_DATA[2]),
	.hps_io_hps_io_emac1_inst_TXD3	(HPS_ENET_TX_DATA[3]),
	.hps_io_hps_io_emac1_inst_RXD0	(HPS_ENET_RX_DATA[0]),
	.hps_io_hps_io_emac1_inst_MDIO	(HPS_ENET_MDIO),
	.hps_io_hps_io_emac1_inst_MDC		(HPS_ENET_MDC),
	.hps_io_hps_io_emac1_inst_RX_CTL	(HPS_ENET_RX_DV),
	.hps_io_hps_io_emac1_inst_TX_CTL	(HPS_ENET_TX_EN),
	.hps_io_hps_io_emac1_inst_RX_CLK	(HPS_ENET_RX_CLK),
	.hps_io_hps_io_emac1_inst_RXD1	(HPS_ENET_RX_DATA[1]),
	.hps_io_hps_io_emac1_inst_RXD2	(HPS_ENET_RX_DATA[2]),
	.hps_io_hps_io_emac1_inst_RXD3	(HPS_ENET_RX_DATA[3]),

	// Flash
	.hps_io_hps_io_qspi_inst_IO0	(HPS_FLASH_DATA[0]),
	.hps_io_hps_io_qspi_inst_IO1	(HPS_FLASH_DATA[1]),
	.hps_io_hps_io_qspi_inst_IO2	(HPS_FLASH_DATA[2]),
	.hps_io_hps_io_qspi_inst_IO3	(HPS_FLASH_DATA[3]),
	.hps_io_hps_io_qspi_inst_SS0	(HPS_FLASH_NCSO),
	.hps_io_hps_io_qspi_inst_CLK	(HPS_FLASH_DCLK),

	// Accelerometer
	.hps_io_hps_io_gpio_inst_GPIO61	(HPS_GSENSOR_INT),

	//.adc_sclk                        (ADC_SCLK),
	//.adc_cs_n                        (ADC_CS_N),
	//.adc_dout                        (ADC_DOUT),
	//.adc_din                         (ADC_DIN),

	// General Purpose I/O
	.hps_io_hps_io_gpio_inst_GPIO40	(HPS_GPIO[0]),
	.hps_io_hps_io_gpio_inst_GPIO41	(HPS_GPIO[1]),

	// I2C
	.hps_io_hps_io_gpio_inst_GPIO48	(HPS_I2C_CONTROL),
	.hps_io_hps_io_i2c0_inst_SDA		(HPS_I2C1_SDAT),
	.hps_io_hps_io_i2c0_inst_SCL		(HPS_I2C1_SCLK),
	.hps_io_hps_io_i2c1_inst_SDA		(HPS_I2C2_SDAT),
	.hps_io_hps_io_i2c1_inst_SCL		(HPS_I2C2_SCLK),

	// Pushbutton
	.hps_io_hps_io_gpio_inst_GPIO54	(HPS_KEY),

	// LED
	.hps_io_hps_io_gpio_inst_GPIO53	(HPS_LED),

	// SD Card
	.hps_io_hps_io_sdio_inst_CMD	(HPS_SD_CMD),
	.hps_io_hps_io_sdio_inst_D0	(HPS_SD_DATA[0]),
	.hps_io_hps_io_sdio_inst_D1	(HPS_SD_DATA[1]),
	.hps_io_hps_io_sdio_inst_CLK	(HPS_SD_CLK),
	.hps_io_hps_io_sdio_inst_D2	(HPS_SD_DATA[2]),
	.hps_io_hps_io_sdio_inst_D3	(HPS_SD_DATA[3]),

	// SPI
	.hps_io_hps_io_spim1_inst_CLK		(HPS_SPIM_CLK),
	.hps_io_hps_io_spim1_inst_MOSI	(HPS_SPIM_MOSI),
	.hps_io_hps_io_spim1_inst_MISO	(HPS_SPIM_MISO),
	.hps_io_hps_io_spim1_inst_SS0		(HPS_SPIM_SS),

	// UART
	.hps_io_hps_io_uart0_inst_RX	(HPS_UART_RX),
	.hps_io_hps_io_uart0_inst_TX	(HPS_UART_TX),

	// USB
	.hps_io_hps_io_gpio_inst_GPIO09	(HPS_CONV_USB_N),
	.hps_io_hps_io_usb1_inst_D0		(HPS_USB_DATA[0]),
	.hps_io_hps_io_usb1_inst_D1		(HPS_USB_DATA[1]),
	.hps_io_hps_io_usb1_inst_D2		(HPS_USB_DATA[2]),
	.hps_io_hps_io_usb1_inst_D3		(HPS_USB_DATA[3]),
	.hps_io_hps_io_usb1_inst_D4		(HPS_USB_DATA[4]),
	.hps_io_hps_io_usb1_inst_D5		(HPS_USB_DATA[5]),
	.hps_io_hps_io_usb1_inst_D6		(HPS_USB_DATA[6]),
	.hps_io_hps_io_usb1_inst_D7		(HPS_USB_DATA[7]),
	.hps_io_hps_io_usb1_inst_CLK		(HPS_USB_CLKOUT),
	.hps_io_hps_io_usb1_inst_STP		(HPS_USB_STP),
	.hps_io_hps_io_usb1_inst_DIR		(HPS_USB_DIR),
	.hps_io_hps_io_usb1_inst_NXT		(HPS_USB_NXT),
	
	.pio_reset_external_connection_export			(pio_reset)
);


endmodule

module signed_mult_27 (out, a, b);
	output 	signed  [26:0]	out;
	input 	signed	[26:0] 	a;
	input 	signed	[26:0] 	b;
	// intermediate full bit length
	wire 	signed	[53:0]	mult_out;
	assign mult_out = a * b; //20 integer bits, 34 floating bits
	// select bits for 10.17 fixed point
	assign out = {mult_out[53], mult_out[42:17]};
endmodule


module FIR_filter_l(x, clk, reset, yn_left, yn_right, done, audio_done);
input signed [26:0] x;
input clk;
input reset;
input audio_done;
output reg signed [26:0] yn_left;
output reg signed [26:0] yn_right;
output reg done;

reg signed[26:0] xn [199:0];
wire signed [26:0] v;
reg signed [26:0] h_arr_left [199:0];
reg signed [26:0] h_arr_right [199:0];
wire signed [26:0] product_left;
reg signed [26:0] accum_left;
wire signed [26:0] product_right;
reg signed [26:0] accum_right;
reg [7:0] counter;
reg first;


signed_mult_27 multl(product_left, xn[counter], h_arr_left[counter]);
signed_mult_27 multr(product_right, xn[counter], h_arr_right[counter]);

//assign v = ( intermediate[0] + intermediate[1] + intermediate[2] + intermediate[3] + intermediate[4] + intermediate[5] + intermediate[6] + intermediate[7] + intermediate[8] + intermediate[9] + intermediate[10] + intermediate[11] + intermediate[12] + intermediate[13] + intermediate[14] + intermediate[15] + intermediate[16] + intermediate[17] + intermediate[18] + intermediate[19] + intermediate[20] + intermediate[21] + intermediate[22] + intermediate[23] + intermediate[24] + intermediate[25] + intermediate[26] + intermediate[27] + intermediate[28] + intermediate[29] + intermediate[30] + intermediate[31] + intermediate[32] + intermediate[33] + intermediate[34] + intermediate[35] + intermediate[36] + intermediate[37] + intermediate[38] + intermediate[39] + intermediate[40] + intermediate[41] + intermediate[42] + intermediate[43] + intermediate[44] + intermediate[45] + intermediate[46] + intermediate[47] + intermediate[48] + intermediate[49] + intermediate[50] + intermediate[51] + intermediate[52] + intermediate[53] + intermediate[54] + intermediate[55] + intermediate[56] + intermediate[57] + intermediate[58] + intermediate[59] + intermediate[60] + intermediate[61] + intermediate[62] + intermediate[63] + intermediate[64] + intermediate[65] + intermediate[66] + intermediate[67] + intermediate[68] + intermediate[69] + intermediate[70] + intermediate[71] + intermediate[72] + intermediate[73] + intermediate[74] + intermediate[75] + intermediate[76] + intermediate[77] + intermediate[78] + intermediate[79] + intermediate[80] + intermediate[81] + intermediate[82] + intermediate[83] + intermediate[84] + intermediate[85] + intermediate[86] + intermediate[87] + intermediate[88] + intermediate[89] + intermediate[90] + intermediate[91] + intermediate[92] + intermediate[93] + intermediate[94] + intermediate[95] + intermediate[96] + intermediate[97] + intermediate[98] + intermediate[99] + intermediate[100] + intermediate[101] + intermediate[102] + intermediate[103] + intermediate[104] + intermediate[105] + intermediate[106] + intermediate[107] + intermediate[108] + intermediate[109] + intermediate[110] + intermediate[111] + intermediate[112] + intermediate[113] + intermediate[114] + intermediate[115] + intermediate[116] + intermediate[117] + intermediate[118] + intermediate[119] + intermediate[120] + intermediate[121] + intermediate[122] + intermediate[123] + intermediate[124] + intermediate[125] + intermediate[126] + intermediate[127] + intermediate[128] + intermediate[129] + intermediate[130] + intermediate[131] + intermediate[132] + intermediate[133] + intermediate[134] + intermediate[135] + intermediate[136] + intermediate[137] + intermediate[138] + intermediate[139] + intermediate[140] + intermediate[141] + intermediate[142] + intermediate[143] + intermediate[144] + intermediate[145] + intermediate[146] + intermediate[147] + intermediate[148] + intermediate[149] + intermediate[150] + intermediate[151] + intermediate[152] + intermediate[153] + intermediate[154] + intermediate[155] + intermediate[156] + intermediate[157] + intermediate[158] + intermediate[159] + intermediate[160] + intermediate[161] + intermediate[162] + intermediate[163] + intermediate[164] + intermediate[165] + intermediate[166] + intermediate[167] + intermediate[168] + intermediate[169] + intermediate[170] + intermediate[171] + intermediate[172] + intermediate[173] + intermediate[174] + intermediate[175] + intermediate[176] + intermediate[177] + intermediate[178] + intermediate[179] + intermediate[180] + intermediate[181] + intermediate[182] + intermediate[183] + intermediate[184] + intermediate[185] + intermediate[186] + intermediate[187] + intermediate[188] + intermediate[189] + intermediate[190] + intermediate[191] + intermediate[192] + intermediate[193] + intermediate[194] + intermediate[195] + intermediate[196] + intermediate[197] + intermediate[198] + intermediate[199]);
always @ (posedge clk) begin
    if(reset) begin
	counter <= 8'd0;
	accum_left <= 8'd0;
	accum_right <= 8'd0;
	done <= 0;
	first <= 1;
        xn[0] <= 27'd0;
        xn[1] <= 27'd0;
        xn[2] <= 27'd0;
        xn[3] <= 27'd0;
        xn[4] <= 27'd0;
        xn[5] <= 27'd0;
        xn[6] <= 27'd0;
        xn[7] <= 27'd0;
        xn[8] <= 27'd0;
        xn[9] <= 27'd0;
        xn[10] <= 27'd0;
        xn[11] <= 27'd0;
        xn[12] <= 27'd0;
        xn[13] <= 27'd0;
        xn[14] <= 27'd0;
        xn[15] <= 27'd0;
        xn[16] <= 27'd0;
        xn[17] <= 27'd0;
        xn[18] <= 27'd0;
        xn[19] <= 27'd0;
        xn[20] <= 27'd0;
        xn[21] <= 27'd0;
        xn[22] <= 27'd0;
        xn[23] <= 27'd0;
        xn[24] <= 27'd0;
        xn[25] <= 27'd0;
        xn[26] <= 27'd0;
        xn[27] <= 27'd0;
        xn[28] <= 27'd0;
        xn[29] <= 27'd0;
        xn[30] <= 27'd0;
        xn[31] <= 27'd0;
        xn[32] <= 27'd0;
        xn[33] <= 27'd0;
        xn[34] <= 27'd0;
        xn[35] <= 27'd0;
        xn[36] <= 27'd0;
        xn[37] <= 27'd0;
        xn[38] <= 27'd0;
        xn[39] <= 27'd0;
        xn[40] <= 27'd0;
        xn[41] <= 27'd0;
        xn[42] <= 27'd0;
        xn[43] <= 27'd0;
        xn[44] <= 27'd0;
        xn[45] <= 27'd0;
        xn[46] <= 27'd0;
        xn[47] <= 27'd0;
        xn[48] <= 27'd0;
        xn[49] <= 27'd0;
        xn[50] <= 27'd0;
        xn[51] <= 27'd0;
        xn[52] <= 27'd0;
        xn[53] <= 27'd0;
        xn[54] <= 27'd0;
        xn[55] <= 27'd0;
        xn[56] <= 27'd0;
        xn[57] <= 27'd0;
        xn[58] <= 27'd0;
        xn[59] <= 27'd0;
        xn[60] <= 27'd0;
        xn[61] <= 27'd0;
        xn[62] <= 27'd0;
        xn[63] <= 27'd0;
        xn[64] <= 27'd0;
        xn[65] <= 27'd0;
        xn[66] <= 27'd0;
        xn[67] <= 27'd0;
        xn[68] <= 27'd0;
        xn[69] <= 27'd0;
        xn[70] <= 27'd0;
        xn[71] <= 27'd0;
        xn[72] <= 27'd0;
        xn[73] <= 27'd0;
        xn[74] <= 27'd0;
        xn[75] <= 27'd0;
        xn[76] <= 27'd0;
        xn[77] <= 27'd0;
        xn[78] <= 27'd0;
        xn[79] <= 27'd0;
        xn[80] <= 27'd0;
        xn[81] <= 27'd0;
        xn[82] <= 27'd0;
        xn[83] <= 27'd0;
        xn[84] <= 27'd0;
        xn[85] <= 27'd0;
        xn[86] <= 27'd0;
        xn[87] <= 27'd0;
        xn[88] <= 27'd0;
        xn[89] <= 27'd0;
        xn[90] <= 27'd0;
        xn[91] <= 27'd0;
        xn[92] <= 27'd0;
        xn[93] <= 27'd0;
        xn[94] <= 27'd0;
        xn[95] <= 27'd0;
        xn[96] <= 27'd0;
        xn[97] <= 27'd0;
        xn[98] <= 27'd0;
        xn[99] <= 27'd0;
        xn[100] <= 27'd0;
        xn[101] <= 27'd0;
        xn[102] <= 27'd0;
        xn[103] <= 27'd0;
        xn[104] <= 27'd0;
        xn[105] <= 27'd0;
        xn[106] <= 27'd0;
        xn[107] <= 27'd0;
        xn[108] <= 27'd0;
        xn[109] <= 27'd0;
        xn[110] <= 27'd0;
        xn[111] <= 27'd0;
        xn[112] <= 27'd0;
        xn[113] <= 27'd0;
        xn[114] <= 27'd0;
        xn[115] <= 27'd0;
        xn[116] <= 27'd0;
        xn[117] <= 27'd0;
        xn[118] <= 27'd0;
        xn[119] <= 27'd0;
        xn[120] <= 27'd0;
        xn[121] <= 27'd0;
        xn[122] <= 27'd0;
        xn[123] <= 27'd0;
        xn[124] <= 27'd0;
        xn[125] <= 27'd0;
        xn[126] <= 27'd0;
        xn[127] <= 27'd0;
        xn[128] <= 27'd0;
        xn[129] <= 27'd0;
        xn[130] <= 27'd0;
        xn[131] <= 27'd0;
        xn[132] <= 27'd0;
        xn[133] <= 27'd0;
        xn[134] <= 27'd0;
        xn[135] <= 27'd0;
        xn[136] <= 27'd0;
        xn[137] <= 27'd0;
        xn[138] <= 27'd0;
        xn[139] <= 27'd0;
        xn[140] <= 27'd0;
        xn[141] <= 27'd0;
        xn[142] <= 27'd0;
        xn[143] <= 27'd0;
        xn[144] <= 27'd0;
        xn[145] <= 27'd0;
        xn[146] <= 27'd0;
        xn[147] <= 27'd0;
        xn[148] <= 27'd0;
        xn[149] <= 27'd0;
        xn[150] <= 27'd0;
        xn[151] <= 27'd0;
        xn[152] <= 27'd0;
        xn[153] <= 27'd0;
        xn[154] <= 27'd0;
        xn[155] <= 27'd0;
        xn[156] <= 27'd0;
        xn[157] <= 27'd0;
        xn[158] <= 27'd0;
        xn[159] <= 27'd0;
        xn[160] <= 27'd0;
        xn[161] <= 27'd0;
        xn[162] <= 27'd0;
        xn[163] <= 27'd0;
        xn[164] <= 27'd0;
        xn[165] <= 27'd0;
        xn[166] <= 27'd0;
        xn[167] <= 27'd0;
        xn[168] <= 27'd0;
        xn[169] <= 27'd0;
        xn[170] <= 27'd0;
        xn[171] <= 27'd0;
        xn[172] <= 27'd0;
        xn[173] <= 27'd0;
        xn[174] <= 27'd0;
        xn[175] <= 27'd0;
        xn[176] <= 27'd0;
        xn[177] <= 27'd0;
        xn[178] <= 27'd0;
        xn[179] <= 27'd0;
        xn[180] <= 27'd0;
        xn[181] <= 27'd0;
        xn[182] <= 27'd0;
        xn[183] <= 27'd0;
        xn[184] <= 27'd0;
        xn[185] <= 27'd0;
        xn[186] <= 27'd0;
        xn[187] <= 27'd0;
        xn[188] <= 27'd0;
        xn[189] <= 27'd0;
        xn[190] <= 27'd0;
        xn[191] <= 27'd0;
        xn[192] <= 27'd0;
        xn[193] <= 27'd0;
        xn[194] <= 27'd0;
        xn[195] <= 27'd0;
        xn[196] <= 27'd0;
        xn[197] <= 27'd0;
        xn[198] <= 27'd0;
        xn[199] <= 27'd0;
        yn_left <= 27'd0;
		  yn_right <= 27'd0;
    end
    else begin
	if(counter == 8'd199) begin
		
		if (audio_done) begin
		done <= 1;
		yn_left <= accum_left;
		accum_left <= 8'd0;
		yn_right <= accum_right;
		accum_right <= 8'd0;	
		counter<=8'd0;
		xn[0] <= x;
        	xn[1] <= xn[0];
        xn[2] <= xn[1];
        xn[3] <= xn[2];
        xn[4] <= xn[3];
        xn[5] <= xn[4];
        xn[6] <= xn[5];
        xn[7] <= xn[6];
        xn[8] <= xn[7];
        xn[9] <= xn[8];
        xn[10] <= xn[9];
        xn[11] <= xn[10];
        xn[12] <= xn[11];
        xn[13] <= xn[12];
        xn[14] <= xn[13];
        xn[15] <= xn[14];
        xn[16] <= xn[15];
        xn[17] <= xn[16];
        xn[18] <= xn[17];
        xn[19] <= xn[18];
        xn[20] <= xn[19];
        xn[21] <= xn[20];
        xn[22] <= xn[21];
        xn[23] <= xn[22];
        xn[24] <= xn[23];
        xn[25] <= xn[24];
        xn[26] <= xn[25];
        xn[27] <= xn[26];
        xn[28] <= xn[27];
        xn[29] <= xn[28];
        xn[30] <= xn[29];
        xn[31] <= xn[30];
        xn[32] <= xn[31];
        xn[33] <= xn[32];
        xn[34] <= xn[33];
        xn[35] <= xn[34];
        xn[36] <= xn[35];
        xn[37] <= xn[36];
        xn[38] <= xn[37];
        xn[39] <= xn[38];
        xn[40] <= xn[39];
        xn[41] <= xn[40];
        xn[42] <= xn[41];
        xn[43] <= xn[42];
        xn[44] <= xn[43];
        xn[45] <= xn[44];
        xn[46] <= xn[45];
        xn[47] <= xn[46];
        xn[48] <= xn[47];
        xn[49] <= xn[48];
        xn[50] <= xn[49];
        xn[51] <= xn[50];
        xn[52] <= xn[51];
        xn[53] <= xn[52];
        xn[54] <= xn[53];
        xn[55] <= xn[54];
        xn[56] <= xn[55];
        xn[57] <= xn[56];
        xn[58] <= xn[57];
        xn[59] <= xn[58];
        xn[60] <= xn[59];
        xn[61] <= xn[60];
        xn[62] <= xn[61];
        xn[63] <= xn[62];
        xn[64] <= xn[63];
        xn[65] <= xn[64];
        xn[66] <= xn[65];
        xn[67] <= xn[66];
        xn[68] <= xn[67];
        xn[69] <= xn[68];
        xn[70] <= xn[69];
        xn[71] <= xn[70];
        xn[72] <= xn[71];
        xn[73] <= xn[72];
        xn[74] <= xn[73];
        xn[75] <= xn[74];
        xn[76] <= xn[75];
        xn[77] <= xn[76];
        xn[78] <= xn[77];
        xn[79] <= xn[78];
        xn[80] <= xn[79];
        xn[81] <= xn[80];
        xn[82] <= xn[81];
        xn[83] <= xn[82];
        xn[84] <= xn[83];
        xn[85] <= xn[84];
        xn[86] <= xn[85];
        xn[87] <= xn[86];
        xn[88] <= xn[87];
        xn[89] <= xn[88];
        xn[90] <= xn[89];
        xn[91] <= xn[90];
        xn[92] <= xn[91];
        xn[93] <= xn[92];
        xn[94] <= xn[93];
        xn[95] <= xn[94];
        xn[96] <= xn[95];
        xn[97] <= xn[96];
        xn[98] <= xn[97];
        xn[99] <= xn[98];
        xn[100] <= xn[99];
        xn[101] <= xn[100];
        xn[102] <= xn[101];
        xn[103] <= xn[102];
        xn[104] <= xn[103];
        xn[105] <= xn[104];
        xn[106] <= xn[105];
        xn[107] <= xn[106];
        xn[108] <= xn[107];
        xn[109] <= xn[108];
        xn[110] <= xn[109];
        xn[111] <= xn[110];
        xn[112] <= xn[111];
        xn[113] <= xn[112];
        xn[114] <= xn[113];
        xn[115] <= xn[114];
        xn[116] <= xn[115];
        xn[117] <= xn[116];
        xn[118] <= xn[117];
        xn[119] <= xn[118];
        xn[120] <= xn[119];
        xn[121] <= xn[120];
        xn[122] <= xn[121];
        xn[123] <= xn[122];
        xn[124] <= xn[123];
        xn[125] <= xn[124];
        xn[126] <= xn[125];
        xn[127] <= xn[126];
        xn[128] <= xn[127];
        xn[129] <= xn[128];
        xn[130] <= xn[129];
        xn[131] <= xn[130];
        xn[132] <= xn[131];
        xn[133] <= xn[132];
        xn[134] <= xn[133];
        xn[135] <= xn[134];
        xn[136] <= xn[135];
        xn[137] <= xn[136];
        xn[138] <= xn[137];
        xn[139] <= xn[138];
        xn[140] <= xn[139];
        xn[141] <= xn[140];
        xn[142] <= xn[141];
        xn[143] <= xn[142];
        xn[144] <= xn[143];
        xn[145] <= xn[144];
        xn[146] <= xn[145];
        xn[147] <= xn[146];
        xn[148] <= xn[147];
        xn[149] <= xn[148];
        xn[150] <= xn[149];
        xn[151] <= xn[150];
        xn[152] <= xn[151];
        xn[153] <= xn[152];
        xn[154] <= xn[153];
        xn[155] <= xn[154];
        xn[156] <= xn[155];
        xn[157] <= xn[156];
        xn[158] <= xn[157];
        xn[159] <= xn[158];
        xn[160] <= xn[159];
        xn[161] <= xn[160];
        xn[162] <= xn[161];
        xn[163] <= xn[162];
        xn[164] <= xn[163];
        xn[165] <= xn[164];
        xn[166] <= xn[165];
        xn[167] <= xn[166];
        xn[168] <= xn[167];
        xn[169] <= xn[168];
        xn[170] <= xn[169];
        xn[171] <= xn[170];
        xn[172] <= xn[171];
        xn[173] <= xn[172];
        xn[174] <= xn[173];
        xn[175] <= xn[174];
        xn[176] <= xn[175];
        xn[177] <= xn[176];
        xn[178] <= xn[177];
        xn[179] <= xn[178];
        xn[180] <= xn[179];
        xn[181] <= xn[180];
        xn[182] <= xn[181];
        xn[183] <= xn[182];
        xn[184] <= xn[183];
        xn[185] <= xn[184];
        xn[186] <= xn[185];
        xn[187] <= xn[186];
        xn[188] <= xn[187];
        xn[189] <= xn[188];
        xn[190] <= xn[189];
        xn[191] <= xn[190];
        xn[192] <= xn[191];
        xn[193] <= xn[192];
        xn[194] <= xn[193];
        xn[195] <= xn[194];
        xn[196] <= xn[195];
        xn[197] <= xn[196];
        xn[198] <= xn[197];
        xn[199] <= xn[198];
		end //end if(audio_done)
	end
	else begin
		accum_left <= accum_left + product_left;
		accum_right <= accum_right + product_right;
		counter <= counter+1;
		done <= 0;
	end
	if(first) begin
    case(counter)
        8'd0 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd1 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd2 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd3 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd4 : begin  h_arr_left[counter] = -27'd5; h_arr_right[counter] = -27'd1; end
        8'd5 : begin  h_arr_left[counter] = -27'd2; h_arr_right[counter] = -27'd7; end
        8'd6 : begin  h_arr_left[counter] = -27'd25; h_arr_right[counter] = -27'd21; end
        8'd7 : begin  h_arr_left[counter] = -27'd46; h_arr_right[counter] = -27'd21; end
        8'd8 : begin  h_arr_left[counter] = -27'd91; h_arr_right[counter] = -27'd63; end
        8'd9 : begin  h_arr_left[counter] = -27'd40; h_arr_right[counter] = -27'd37; end
        8'd10 : begin  h_arr_left[counter] = -27'd231; h_arr_right[counter] = -27'd58; end
        8'd11 : begin  h_arr_left[counter] = -27'd8; h_arr_right[counter] = -27'd80; end
        8'd12 : begin  h_arr_left[counter] = 27'd88; h_arr_right[counter] = -27'd120; end
        8'd13 : begin  h_arr_left[counter] = 27'd130; h_arr_right[counter] = -27'd83; end
        8'd14 : begin  h_arr_left[counter] = 27'd4014; h_arr_right[counter] = -27'd41; end
        8'd15 : begin  h_arr_left[counter] = -27'd2521; h_arr_right[counter] = -27'd205; end
        8'd16 : begin  h_arr_left[counter] = -27'd4145; h_arr_right[counter] = -27'd109; end
        8'd17 : begin  h_arr_left[counter] = 27'd7158; h_arr_right[counter] = -27'd137; end
        8'd18 : begin  h_arr_left[counter] = -27'd2868; h_arr_right[counter] = -27'd152; end
        8'd19 : begin  h_arr_left[counter] = -27'd2458; h_arr_right[counter] = -27'd65; end
        8'd20 : begin  h_arr_left[counter] = 27'd614; h_arr_right[counter] = -27'd157; end
        8'd21 : begin  h_arr_left[counter] = -27'd1236; h_arr_right[counter] = -27'd202; end
        8'd22 : begin  h_arr_left[counter] = -27'd1427; h_arr_right[counter] = -27'd198; end
        8'd23 : begin  h_arr_left[counter] = 27'd14028; h_arr_right[counter] = -27'd59; end
        8'd24 : begin  h_arr_left[counter] = 27'd170181; h_arr_right[counter] = -27'd109; end
        8'd25 : begin  h_arr_left[counter] = 27'd134281; h_arr_right[counter] = -27'd290; end
        8'd26 : begin  h_arr_left[counter] = -27'd8856; h_arr_right[counter] = -27'd235; end
        8'd27 : begin  h_arr_left[counter] = -27'd91313; h_arr_right[counter] = -27'd205; end
        8'd28 : begin  h_arr_left[counter] = -27'd122821; h_arr_right[counter] = -27'd97; end
        8'd29 : begin  h_arr_left[counter] = -27'd44667; h_arr_right[counter] = -27'd191; end
        8'd30 : begin  h_arr_left[counter] = -27'd114286; h_arr_right[counter] = -27'd115; end
        8'd31 : begin  h_arr_left[counter] = 27'd20596; h_arr_right[counter] = 27'd39; end
        8'd32 : begin  h_arr_left[counter] = 27'd131106; h_arr_right[counter] = -27'd240; end
        8'd33 : begin  h_arr_left[counter] = 27'd22; h_arr_right[counter] = -27'd299; end
        8'd34 : begin  h_arr_left[counter] = -27'd11180; h_arr_right[counter] = -27'd153; end
        8'd35 : begin  h_arr_left[counter] = -27'd8841; h_arr_right[counter] = -27'd96; end
        8'd36 : begin  h_arr_left[counter] = -27'd10026; h_arr_right[counter] = -27'd247; end
        8'd37 : begin  h_arr_left[counter] = -27'd8264; h_arr_right[counter] = 27'd38; end
        8'd38 : begin  h_arr_left[counter] = -27'd30851; h_arr_right[counter] = 27'd36; end
        8'd39 : begin  h_arr_left[counter] = -27'd26645; h_arr_right[counter] = -27'd161; end
        8'd40 : begin  h_arr_left[counter] = 27'd2695; h_arr_right[counter] = 27'd24; end
        8'd41 : begin  h_arr_left[counter] = 27'd6176; h_arr_right[counter] = -27'd196; end
        8'd42 : begin  h_arr_left[counter] = -27'd351; h_arr_right[counter] = -27'd141; end
        8'd43 : begin  h_arr_left[counter] = 27'd4408; h_arr_right[counter] = -27'd141; end
        8'd44 : begin  h_arr_left[counter] = -27'd21950; h_arr_right[counter] = -27'd382; end
        8'd45 : begin  h_arr_left[counter] = -27'd27297; h_arr_right[counter] = -27'd184; end
        8'd46 : begin  h_arr_left[counter] = 27'd5656; h_arr_right[counter] = -27'd252; end
        8'd47 : begin  h_arr_left[counter] = 27'd11622; h_arr_right[counter] = -27'd639; end
        8'd48 : begin  h_arr_left[counter] = -27'd12748; h_arr_right[counter] = -27'd314; end
        8'd49 : begin  h_arr_left[counter] = -27'd11842; h_arr_right[counter] = -27'd79; end
        8'd50 : begin  h_arr_left[counter] = -27'd5686; h_arr_right[counter] = 27'd1410; end
        8'd51 : begin  h_arr_left[counter] = -27'd2390; h_arr_right[counter] = 27'd7815; end
        8'd52 : begin  h_arr_left[counter] = -27'd6466; h_arr_right[counter] = 27'd13550; end
        8'd53 : begin  h_arr_left[counter] = -27'd13784; h_arr_right[counter] = 27'd15230; end
        8'd54 : begin  h_arr_left[counter] = -27'd6010; h_arr_right[counter] = 27'd15711; end
        8'd55 : begin  h_arr_left[counter] = -27'd103; h_arr_right[counter] = 27'd13358; end
        8'd56 : begin  h_arr_left[counter] = 27'd7482; h_arr_right[counter] = 27'd5350; end
        8'd57 : begin  h_arr_left[counter] = 27'd6201; h_arr_right[counter] = -27'd3290; end
        8'd58 : begin  h_arr_left[counter] = 27'd2870; h_arr_right[counter] = -27'd5098; end
        8'd59 : begin  h_arr_left[counter] = -27'd990; h_arr_right[counter] = -27'd4895; end
        8'd60 : begin  h_arr_left[counter] = -27'd5557; h_arr_right[counter] = -27'd4745; end
        8'd61 : begin  h_arr_left[counter] = 27'd851; h_arr_right[counter] = -27'd493; end
        8'd62 : begin  h_arr_left[counter] = -27'd476; h_arr_right[counter] = 27'd6603; end
        8'd63 : begin  h_arr_left[counter] = -27'd7166; h_arr_right[counter] = 27'd7034; end
        8'd64 : begin  h_arr_left[counter] = -27'd3731; h_arr_right[counter] = -27'd1810; end
        8'd65 : begin  h_arr_left[counter] = 27'd4160; h_arr_right[counter] = -27'd6102; end
        8'd66 : begin  h_arr_left[counter] = 27'd6303; h_arr_right[counter] = -27'd6619; end
        8'd67 : begin  h_arr_left[counter] = 27'd152; h_arr_right[counter] = -27'd8874; end
        8'd68 : begin  h_arr_left[counter] = -27'd2191; h_arr_right[counter] = -27'd8998; end
        8'd69 : begin  h_arr_left[counter] = 27'd216; h_arr_right[counter] = -27'd9002; end
        8'd70 : begin  h_arr_left[counter] = 27'd992; h_arr_right[counter] = -27'd8699; end
        8'd71 : begin  h_arr_left[counter] = 27'd1247; h_arr_right[counter] = -27'd6690; end
        8'd72 : begin  h_arr_left[counter] = -27'd423; h_arr_right[counter] = -27'd2310; end
        8'd73 : begin  h_arr_left[counter] = -27'd3336; h_arr_right[counter] = 27'd729; end
        8'd74 : begin  h_arr_left[counter] = -27'd1277; h_arr_right[counter] = -27'd1124; end
        8'd75 : begin  h_arr_left[counter] = 27'd1468; h_arr_right[counter] = -27'd3004; end
        8'd76 : begin  h_arr_left[counter] = 27'd2773; h_arr_right[counter] = -27'd2782; end
        8'd77 : begin  h_arr_left[counter] = 27'd3464; h_arr_right[counter] = -27'd2270; end
        8'd78 : begin  h_arr_left[counter] = -27'd1051; h_arr_right[counter] = -27'd4190; end
        8'd79 : begin  h_arr_left[counter] = -27'd1358; h_arr_right[counter] = -27'd5661; end
        8'd80 : begin  h_arr_left[counter] = 27'd3325; h_arr_right[counter] = -27'd6404; end
        8'd81 : begin  h_arr_left[counter] = 27'd3256; h_arr_right[counter] = -27'd8345; end
        8'd82 : begin  h_arr_left[counter] = 27'd2616; h_arr_right[counter] = -27'd5801; end
        8'd83 : begin  h_arr_left[counter] = 27'd1746; h_arr_right[counter] = -27'd3075; end
        8'd84 : begin  h_arr_left[counter] = 27'd4207; h_arr_right[counter] = -27'd3693; end
        8'd85 : begin  h_arr_left[counter] = 27'd5403; h_arr_right[counter] = -27'd3292; end
        8'd86 : begin  h_arr_left[counter] = 27'd1584; h_arr_right[counter] = -27'd2971; end
        8'd87 : begin  h_arr_left[counter] = -27'd99; h_arr_right[counter] = -27'd2358; end
        8'd88 : begin  h_arr_left[counter] = 27'd218; h_arr_right[counter] = -27'd956; end
        8'd89 : begin  h_arr_left[counter] = -27'd980; h_arr_right[counter] = -27'd965; end
        8'd90 : begin  h_arr_left[counter] = -27'd1877; h_arr_right[counter] = -27'd2035; end
        8'd91 : begin  h_arr_left[counter] = -27'd1264; h_arr_right[counter] = -27'd1023; end
        8'd92 : begin  h_arr_left[counter] = -27'd958; h_arr_right[counter] = 27'd968; end
        8'd93 : begin  h_arr_left[counter] = -27'd1249; h_arr_right[counter] = 27'd370; end
        8'd94 : begin  h_arr_left[counter] = 27'd485; h_arr_right[counter] = -27'd189; end
        8'd95 : begin  h_arr_left[counter] = 27'd2675; h_arr_right[counter] = -27'd503; end
        8'd96 : begin  h_arr_left[counter] = 27'd2148; h_arr_right[counter] = -27'd931; end
        8'd97 : begin  h_arr_left[counter] = 27'd164; h_arr_right[counter] = -27'd44; end
        8'd98 : begin  h_arr_left[counter] = -27'd1909; h_arr_right[counter] = -27'd396; end
        8'd99 : begin  h_arr_left[counter] = -27'd1251; h_arr_right[counter] = -27'd1574; end
        8'd100 : begin  h_arr_left[counter] = 27'd271; h_arr_right[counter] = -27'd1456; end
        8'd101 : begin  h_arr_left[counter] = -27'd742; h_arr_right[counter] = -27'd599; end
        8'd102 : begin  h_arr_left[counter] = -27'd103; h_arr_right[counter] = 27'd78; end
        8'd103 : begin  h_arr_left[counter] = 27'd993; h_arr_right[counter] = 27'd99; end
        8'd104 : begin  h_arr_left[counter] = 27'd875; h_arr_right[counter] = -27'd266; end
        8'd105 : begin  h_arr_left[counter] = 27'd1062; h_arr_right[counter] = -27'd586; end
        8'd106 : begin  h_arr_left[counter] = 27'd720; h_arr_right[counter] = -27'd511; end
        8'd107 : begin  h_arr_left[counter] = 27'd1220; h_arr_right[counter] = -27'd294; end
        8'd108 : begin  h_arr_left[counter] = 27'd849; h_arr_right[counter] = -27'd971; end
        8'd109 : begin  h_arr_left[counter] = -27'd572; h_arr_right[counter] = -27'd1763; end
        8'd110 : begin  h_arr_left[counter] = -27'd370; h_arr_right[counter] = -27'd1519; end
        8'd111 : begin  h_arr_left[counter] = 27'd734; h_arr_right[counter] = -27'd758; end
        8'd112 : begin  h_arr_left[counter] = 27'd1762; h_arr_right[counter] = 27'd224; end
        8'd113 : begin  h_arr_left[counter] = 27'd1042; h_arr_right[counter] = 27'd305; end
        8'd114 : begin  h_arr_left[counter] = 27'd2070; h_arr_right[counter] = 27'd340; end
        8'd115 : begin  h_arr_left[counter] = 27'd2876; h_arr_right[counter] = 27'd1007; end
        8'd116 : begin  h_arr_left[counter] = 27'd851; h_arr_right[counter] = 27'd1401; end
        8'd117 : begin  h_arr_left[counter] = -27'd77; h_arr_right[counter] = 27'd1043; end
        8'd118 : begin  h_arr_left[counter] = 27'd129; h_arr_right[counter] = 27'd406; end
        8'd119 : begin  h_arr_left[counter] = 27'd1885; h_arr_right[counter] = -27'd214; end
        8'd120 : begin  h_arr_left[counter] = 27'd2466; h_arr_right[counter] = -27'd672; end
        8'd121 : begin  h_arr_left[counter] = 27'd1405; h_arr_right[counter] = -27'd243; end
        8'd122 : begin  h_arr_left[counter] = 27'd3109; h_arr_right[counter] = -27'd139; end
        8'd123 : begin  h_arr_left[counter] = 27'd3027; h_arr_right[counter] = -27'd559; end
        8'd124 : begin  h_arr_left[counter] = 27'd155; h_arr_right[counter] = 27'd124; end
        8'd125 : begin  h_arr_left[counter] = -27'd1191; h_arr_right[counter] = 27'd1574; end
        8'd126 : begin  h_arr_left[counter] = -27'd1664; h_arr_right[counter] = 27'd2511; end
        8'd127 : begin  h_arr_left[counter] = -27'd2108; h_arr_right[counter] = 27'd2804; end
        8'd128 : begin  h_arr_left[counter] = -27'd2829; h_arr_right[counter] = 27'd2393; end
        8'd129 : begin  h_arr_left[counter] = -27'd2558; h_arr_right[counter] = 27'd1844; end
        8'd130 : begin  h_arr_left[counter] = -27'd1345; h_arr_right[counter] = 27'd1383; end
        8'd131 : begin  h_arr_left[counter] = -27'd371; h_arr_right[counter] = 27'd1038; end
        8'd132 : begin  h_arr_left[counter] = 27'd658; h_arr_right[counter] = 27'd344; end
        8'd133 : begin  h_arr_left[counter] = 27'd2155; h_arr_right[counter] = -27'd30; end
        8'd134 : begin  h_arr_left[counter] = 27'd3856; h_arr_right[counter] = 27'd404; end
        8'd135 : begin  h_arr_left[counter] = 27'd2286; h_arr_right[counter] = 27'd693; end
        8'd136 : begin  h_arr_left[counter] = -27'd342; h_arr_right[counter] = 27'd732; end
        8'd137 : begin  h_arr_left[counter] = -27'd751; h_arr_right[counter] = 27'd735; end
        8'd138 : begin  h_arr_left[counter] = -27'd830; h_arr_right[counter] = 27'd445; end
        8'd139 : begin  h_arr_left[counter] = -27'd695; h_arr_right[counter] = 27'd382; end
        8'd140 : begin  h_arr_left[counter] = -27'd1638; h_arr_right[counter] = 27'd353; end
        8'd141 : begin  h_arr_left[counter] = -27'd719; h_arr_right[counter] = 27'd87; end
        8'd142 : begin  h_arr_left[counter] = 27'd1180; h_arr_right[counter] = -27'd161; end
        8'd143 : begin  h_arr_left[counter] = -27'd1247; h_arr_right[counter] = -27'd541; end
        8'd144 : begin  h_arr_left[counter] = -27'd1074; h_arr_right[counter] = -27'd633; end
        8'd145 : begin  h_arr_left[counter] = 27'd1651; h_arr_right[counter] = -27'd410; end
        8'd146 : begin  h_arr_left[counter] = 27'd1097; h_arr_right[counter] = -27'd130; end
        8'd147 : begin  h_arr_left[counter] = -27'd573; h_arr_right[counter] = -27'd26; end
        8'd148 : begin  h_arr_left[counter] = -27'd1012; h_arr_right[counter] = 27'd265; end
        8'd149 : begin  h_arr_left[counter] = -27'd206; h_arr_right[counter] = 27'd804; end
        8'd150 : begin  h_arr_left[counter] = -27'd135; h_arr_right[counter] = 27'd1067; end
        8'd151 : begin  h_arr_left[counter] = 27'd255; h_arr_right[counter] = 27'd1096; end
        8'd152 : begin  h_arr_left[counter] = 27'd1347; h_arr_right[counter] = 27'd1081; end
        8'd153 : begin  h_arr_left[counter] = 27'd2042; h_arr_right[counter] = 27'd937; end
        8'd154 : begin  h_arr_left[counter] = 27'd919; h_arr_right[counter] = 27'd851; end
        8'd155 : begin  h_arr_left[counter] = 27'd104; h_arr_right[counter] = 27'd645; end
        8'd156 : begin  h_arr_left[counter] = 27'd990; h_arr_right[counter] = 27'd366; end
        8'd157 : begin  h_arr_left[counter] = 27'd681; h_arr_right[counter] = 27'd80; end
        8'd158 : begin  h_arr_left[counter] = 27'd219; h_arr_right[counter] = -27'd188; end
        8'd159 : begin  h_arr_left[counter] = 27'd408; h_arr_right[counter] = -27'd192; end
        8'd160 : begin  h_arr_left[counter] = 27'd527; h_arr_right[counter] = -27'd211; end
        8'd161 : begin  h_arr_left[counter] = 27'd199; h_arr_right[counter] = -27'd251; end
        8'd162 : begin  h_arr_left[counter] = -27'd34; h_arr_right[counter] = -27'd141; end
        8'd163 : begin  h_arr_left[counter] = 27'd296; h_arr_right[counter] = 27'd0; end
        8'd164 : begin  h_arr_left[counter] = 27'd398; h_arr_right[counter] = 27'd154; end
        8'd165 : begin  h_arr_left[counter] = 27'd66; h_arr_right[counter] = 27'd262; end
        8'd166 : begin  h_arr_left[counter] = -27'd301; h_arr_right[counter] = 27'd379; end
        8'd167 : begin  h_arr_left[counter] = -27'd141; h_arr_right[counter] = 27'd589; end
        8'd168 : begin  h_arr_left[counter] = 27'd136; h_arr_right[counter] = 27'd695; end
        8'd169 : begin  h_arr_left[counter] = 27'd179; h_arr_right[counter] = 27'd691; end
        8'd170 : begin  h_arr_left[counter] = 27'd100; h_arr_right[counter] = 27'd703; end
        8'd171 : begin  h_arr_left[counter] = -27'd100; h_arr_right[counter] = 27'd681; end
        8'd172 : begin  h_arr_left[counter] = -27'd221; h_arr_right[counter] = 27'd566; end
        8'd173 : begin  h_arr_left[counter] = -27'd385; h_arr_right[counter] = 27'd549; end
        8'd174 : begin  h_arr_left[counter] = -27'd315; h_arr_right[counter] = 27'd558; end
        8'd175 : begin  h_arr_left[counter] = -27'd84; h_arr_right[counter] = 27'd481; end
        8'd176 : begin  h_arr_left[counter] = -27'd88; h_arr_right[counter] = 27'd410; end
        8'd177 : begin  h_arr_left[counter] = -27'd109; h_arr_right[counter] = 27'd407; end
        8'd178 : begin  h_arr_left[counter] = -27'd102; h_arr_right[counter] = 27'd398; end
        8'd179 : begin  h_arr_left[counter] = -27'd9; h_arr_right[counter] = 27'd346; end
        8'd180 : begin  h_arr_left[counter] = -27'd91; h_arr_right[counter] = 27'd260; end
        8'd181 : begin  h_arr_left[counter] = -27'd226; h_arr_right[counter] = 27'd170; end
        8'd182 : begin  h_arr_left[counter] = -27'd80; h_arr_right[counter] = 27'd130; end
        8'd183 : begin  h_arr_left[counter] = -27'd51; h_arr_right[counter] = 27'd106; end
        8'd184 : begin  h_arr_left[counter] = -27'd262; h_arr_right[counter] = 27'd87; end
        8'd185 : begin  h_arr_left[counter] = -27'd388; h_arr_right[counter] = 27'd76; end
        8'd186 : begin  h_arr_left[counter] = -27'd206; h_arr_right[counter] = 27'd80; end
        8'd187 : begin  h_arr_left[counter] = 27'd38; h_arr_right[counter] = 27'd78; end
        8'd188 : begin  h_arr_left[counter] = 27'd30; h_arr_right[counter] = 27'd78; end
        8'd189 : begin  h_arr_left[counter] = -27'd16; h_arr_right[counter] = 27'd74; end
        8'd190 : begin  h_arr_left[counter] = 27'd13; h_arr_right[counter] = 27'd63; end
        8'd191 : begin  h_arr_left[counter] = -27'd8; h_arr_right[counter] = 27'd53; end
        8'd192 : begin  h_arr_left[counter] = -27'd57; h_arr_right[counter] = 27'd46; end
        8'd193 : begin  h_arr_left[counter] = -27'd57; h_arr_right[counter] = 27'd39; end
        8'd194 : begin  h_arr_left[counter] = -27'd20; h_arr_right[counter] = 27'd33; end
        8'd195 : begin  h_arr_left[counter] = -27'd16; h_arr_right[counter] = 27'd26; end
        8'd196 : begin  h_arr_left[counter] = -27'd25; h_arr_right[counter] = 27'd20; end
        8'd197 : begin  h_arr_left[counter] = -27'd6; h_arr_right[counter] = 27'd14; end
        8'd198 : begin  h_arr_left[counter] = 27'd1; h_arr_right[counter] = 27'd8; end
        8'd199 : begin  h_arr_left[counter] = -27'd1; h_arr_right[counter] = 27'd4; first = 0; end
    endcase
    end
        
    end
end
endmodule

module FIR_filter_r(x, clk, reset, yn_left, yn_right, done, audio_done);
input signed [26:0] x;
input clk;
input reset;
input audio_done;
output reg signed [26:0] yn_left;
output reg signed [26:0] yn_right;
output reg done;

reg signed[26:0] xn [199:0];
wire signed [26:0] v;
reg signed [26:0] h_arr_left [199:0];
reg signed [26:0] h_arr_right [199:0];
wire signed [26:0] product_left;
reg signed [26:0] accum_left;
wire signed [26:0] product_right;
reg signed [26:0] accum_right;
reg [7:0] counter;
reg first;


signed_mult_27 multl(product_left, xn[counter], h_arr_left[counter]);
signed_mult_27 multr(product_right, xn[counter], h_arr_right[counter]);

//assign v = ( intermediate[0] + intermediate[1] + intermediate[2] + intermediate[3] + intermediate[4] + intermediate[5] + intermediate[6] + intermediate[7] + intermediate[8] + intermediate[9] + intermediate[10] + intermediate[11] + intermediate[12] + intermediate[13] + intermediate[14] + intermediate[15] + intermediate[16] + intermediate[17] + intermediate[18] + intermediate[19] + intermediate[20] + intermediate[21] + intermediate[22] + intermediate[23] + intermediate[24] + intermediate[25] + intermediate[26] + intermediate[27] + intermediate[28] + intermediate[29] + intermediate[30] + intermediate[31] + intermediate[32] + intermediate[33] + intermediate[34] + intermediate[35] + intermediate[36] + intermediate[37] + intermediate[38] + intermediate[39] + intermediate[40] + intermediate[41] + intermediate[42] + intermediate[43] + intermediate[44] + intermediate[45] + intermediate[46] + intermediate[47] + intermediate[48] + intermediate[49] + intermediate[50] + intermediate[51] + intermediate[52] + intermediate[53] + intermediate[54] + intermediate[55] + intermediate[56] + intermediate[57] + intermediate[58] + intermediate[59] + intermediate[60] + intermediate[61] + intermediate[62] + intermediate[63] + intermediate[64] + intermediate[65] + intermediate[66] + intermediate[67] + intermediate[68] + intermediate[69] + intermediate[70] + intermediate[71] + intermediate[72] + intermediate[73] + intermediate[74] + intermediate[75] + intermediate[76] + intermediate[77] + intermediate[78] + intermediate[79] + intermediate[80] + intermediate[81] + intermediate[82] + intermediate[83] + intermediate[84] + intermediate[85] + intermediate[86] + intermediate[87] + intermediate[88] + intermediate[89] + intermediate[90] + intermediate[91] + intermediate[92] + intermediate[93] + intermediate[94] + intermediate[95] + intermediate[96] + intermediate[97] + intermediate[98] + intermediate[99] + intermediate[100] + intermediate[101] + intermediate[102] + intermediate[103] + intermediate[104] + intermediate[105] + intermediate[106] + intermediate[107] + intermediate[108] + intermediate[109] + intermediate[110] + intermediate[111] + intermediate[112] + intermediate[113] + intermediate[114] + intermediate[115] + intermediate[116] + intermediate[117] + intermediate[118] + intermediate[119] + intermediate[120] + intermediate[121] + intermediate[122] + intermediate[123] + intermediate[124] + intermediate[125] + intermediate[126] + intermediate[127] + intermediate[128] + intermediate[129] + intermediate[130] + intermediate[131] + intermediate[132] + intermediate[133] + intermediate[134] + intermediate[135] + intermediate[136] + intermediate[137] + intermediate[138] + intermediate[139] + intermediate[140] + intermediate[141] + intermediate[142] + intermediate[143] + intermediate[144] + intermediate[145] + intermediate[146] + intermediate[147] + intermediate[148] + intermediate[149] + intermediate[150] + intermediate[151] + intermediate[152] + intermediate[153] + intermediate[154] + intermediate[155] + intermediate[156] + intermediate[157] + intermediate[158] + intermediate[159] + intermediate[160] + intermediate[161] + intermediate[162] + intermediate[163] + intermediate[164] + intermediate[165] + intermediate[166] + intermediate[167] + intermediate[168] + intermediate[169] + intermediate[170] + intermediate[171] + intermediate[172] + intermediate[173] + intermediate[174] + intermediate[175] + intermediate[176] + intermediate[177] + intermediate[178] + intermediate[179] + intermediate[180] + intermediate[181] + intermediate[182] + intermediate[183] + intermediate[184] + intermediate[185] + intermediate[186] + intermediate[187] + intermediate[188] + intermediate[189] + intermediate[190] + intermediate[191] + intermediate[192] + intermediate[193] + intermediate[194] + intermediate[195] + intermediate[196] + intermediate[197] + intermediate[198] + intermediate[199]);
always @ (posedge clk) begin
    if(reset) begin
	counter <= 8'd0;
	accum_left <= 8'd0;
	accum_right <= 8'd0;
	done <= 0;
	first <= 1;
        xn[0] <= 27'd0;
        xn[1] <= 27'd0;
        xn[2] <= 27'd0;
        xn[3] <= 27'd0;
        xn[4] <= 27'd0;
        xn[5] <= 27'd0;
        xn[6] <= 27'd0;
        xn[7] <= 27'd0;
        xn[8] <= 27'd0;
        xn[9] <= 27'd0;
        xn[10] <= 27'd0;
        xn[11] <= 27'd0;
        xn[12] <= 27'd0;
        xn[13] <= 27'd0;
        xn[14] <= 27'd0;
        xn[15] <= 27'd0;
        xn[16] <= 27'd0;
        xn[17] <= 27'd0;
        xn[18] <= 27'd0;
        xn[19] <= 27'd0;
        xn[20] <= 27'd0;
        xn[21] <= 27'd0;
        xn[22] <= 27'd0;
        xn[23] <= 27'd0;
        xn[24] <= 27'd0;
        xn[25] <= 27'd0;
        xn[26] <= 27'd0;
        xn[27] <= 27'd0;
        xn[28] <= 27'd0;
        xn[29] <= 27'd0;
        xn[30] <= 27'd0;
        xn[31] <= 27'd0;
        xn[32] <= 27'd0;
        xn[33] <= 27'd0;
        xn[34] <= 27'd0;
        xn[35] <= 27'd0;
        xn[36] <= 27'd0;
        xn[37] <= 27'd0;
        xn[38] <= 27'd0;
        xn[39] <= 27'd0;
        xn[40] <= 27'd0;
        xn[41] <= 27'd0;
        xn[42] <= 27'd0;
        xn[43] <= 27'd0;
        xn[44] <= 27'd0;
        xn[45] <= 27'd0;
        xn[46] <= 27'd0;
        xn[47] <= 27'd0;
        xn[48] <= 27'd0;
        xn[49] <= 27'd0;
        xn[50] <= 27'd0;
        xn[51] <= 27'd0;
        xn[52] <= 27'd0;
        xn[53] <= 27'd0;
        xn[54] <= 27'd0;
        xn[55] <= 27'd0;
        xn[56] <= 27'd0;
        xn[57] <= 27'd0;
        xn[58] <= 27'd0;
        xn[59] <= 27'd0;
        xn[60] <= 27'd0;
        xn[61] <= 27'd0;
        xn[62] <= 27'd0;
        xn[63] <= 27'd0;
        xn[64] <= 27'd0;
        xn[65] <= 27'd0;
        xn[66] <= 27'd0;
        xn[67] <= 27'd0;
        xn[68] <= 27'd0;
        xn[69] <= 27'd0;
        xn[70] <= 27'd0;
        xn[71] <= 27'd0;
        xn[72] <= 27'd0;
        xn[73] <= 27'd0;
        xn[74] <= 27'd0;
        xn[75] <= 27'd0;
        xn[76] <= 27'd0;
        xn[77] <= 27'd0;
        xn[78] <= 27'd0;
        xn[79] <= 27'd0;
        xn[80] <= 27'd0;
        xn[81] <= 27'd0;
        xn[82] <= 27'd0;
        xn[83] <= 27'd0;
        xn[84] <= 27'd0;
        xn[85] <= 27'd0;
        xn[86] <= 27'd0;
        xn[87] <= 27'd0;
        xn[88] <= 27'd0;
        xn[89] <= 27'd0;
        xn[90] <= 27'd0;
        xn[91] <= 27'd0;
        xn[92] <= 27'd0;
        xn[93] <= 27'd0;
        xn[94] <= 27'd0;
        xn[95] <= 27'd0;
        xn[96] <= 27'd0;
        xn[97] <= 27'd0;
        xn[98] <= 27'd0;
        xn[99] <= 27'd0;
        xn[100] <= 27'd0;
        xn[101] <= 27'd0;
        xn[102] <= 27'd0;
        xn[103] <= 27'd0;
        xn[104] <= 27'd0;
        xn[105] <= 27'd0;
        xn[106] <= 27'd0;
        xn[107] <= 27'd0;
        xn[108] <= 27'd0;
        xn[109] <= 27'd0;
        xn[110] <= 27'd0;
        xn[111] <= 27'd0;
        xn[112] <= 27'd0;
        xn[113] <= 27'd0;
        xn[114] <= 27'd0;
        xn[115] <= 27'd0;
        xn[116] <= 27'd0;
        xn[117] <= 27'd0;
        xn[118] <= 27'd0;
        xn[119] <= 27'd0;
        xn[120] <= 27'd0;
        xn[121] <= 27'd0;
        xn[122] <= 27'd0;
        xn[123] <= 27'd0;
        xn[124] <= 27'd0;
        xn[125] <= 27'd0;
        xn[126] <= 27'd0;
        xn[127] <= 27'd0;
        xn[128] <= 27'd0;
        xn[129] <= 27'd0;
        xn[130] <= 27'd0;
        xn[131] <= 27'd0;
        xn[132] <= 27'd0;
        xn[133] <= 27'd0;
        xn[134] <= 27'd0;
        xn[135] <= 27'd0;
        xn[136] <= 27'd0;
        xn[137] <= 27'd0;
        xn[138] <= 27'd0;
        xn[139] <= 27'd0;
        xn[140] <= 27'd0;
        xn[141] <= 27'd0;
        xn[142] <= 27'd0;
        xn[143] <= 27'd0;
        xn[144] <= 27'd0;
        xn[145] <= 27'd0;
        xn[146] <= 27'd0;
        xn[147] <= 27'd0;
        xn[148] <= 27'd0;
        xn[149] <= 27'd0;
        xn[150] <= 27'd0;
        xn[151] <= 27'd0;
        xn[152] <= 27'd0;
        xn[153] <= 27'd0;
        xn[154] <= 27'd0;
        xn[155] <= 27'd0;
        xn[156] <= 27'd0;
        xn[157] <= 27'd0;
        xn[158] <= 27'd0;
        xn[159] <= 27'd0;
        xn[160] <= 27'd0;
        xn[161] <= 27'd0;
        xn[162] <= 27'd0;
        xn[163] <= 27'd0;
        xn[164] <= 27'd0;
        xn[165] <= 27'd0;
        xn[166] <= 27'd0;
        xn[167] <= 27'd0;
        xn[168] <= 27'd0;
        xn[169] <= 27'd0;
        xn[170] <= 27'd0;
        xn[171] <= 27'd0;
        xn[172] <= 27'd0;
        xn[173] <= 27'd0;
        xn[174] <= 27'd0;
        xn[175] <= 27'd0;
        xn[176] <= 27'd0;
        xn[177] <= 27'd0;
        xn[178] <= 27'd0;
        xn[179] <= 27'd0;
        xn[180] <= 27'd0;
        xn[181] <= 27'd0;
        xn[182] <= 27'd0;
        xn[183] <= 27'd0;
        xn[184] <= 27'd0;
        xn[185] <= 27'd0;
        xn[186] <= 27'd0;
        xn[187] <= 27'd0;
        xn[188] <= 27'd0;
        xn[189] <= 27'd0;
        xn[190] <= 27'd0;
        xn[191] <= 27'd0;
        xn[192] <= 27'd0;
        xn[193] <= 27'd0;
        xn[194] <= 27'd0;
        xn[195] <= 27'd0;
        xn[196] <= 27'd0;
        xn[197] <= 27'd0;
        xn[198] <= 27'd0;
        xn[199] <= 27'd0;
        yn_left <= 27'd0;
		  yn_right <= 27'd0;
    end
    else begin
	if(counter == 8'd199) begin
		
		if (audio_done) begin
		done <= 1;
		yn_left <= accum_left;
		accum_left <= 8'd0;
		yn_right <= accum_right;
		accum_right <= 8'd0;	
		counter<=8'd0;
		xn[0] <= x;
        xn[1] <= xn[0];
        xn[2] <= xn[1];
        xn[3] <= xn[2];
        xn[4] <= xn[3];
        xn[5] <= xn[4];
        xn[6] <= xn[5];
        xn[7] <= xn[6];
        xn[8] <= xn[7];
        xn[9] <= xn[8];
        xn[10] <= xn[9];
        xn[11] <= xn[10];
        xn[12] <= xn[11];
        xn[13] <= xn[12];
        xn[14] <= xn[13];
        xn[15] <= xn[14];
        xn[16] <= xn[15];
        xn[17] <= xn[16];
        xn[18] <= xn[17];
        xn[19] <= xn[18];
        xn[20] <= xn[19];
        xn[21] <= xn[20];
        xn[22] <= xn[21];
        xn[23] <= xn[22];
        xn[24] <= xn[23];
        xn[25] <= xn[24];
        xn[26] <= xn[25];
        xn[27] <= xn[26];
        xn[28] <= xn[27];
        xn[29] <= xn[28];
        xn[30] <= xn[29];
        xn[31] <= xn[30];
        xn[32] <= xn[31];
        xn[33] <= xn[32];
        xn[34] <= xn[33];
        xn[35] <= xn[34];
        xn[36] <= xn[35];
        xn[37] <= xn[36];
        xn[38] <= xn[37];
        xn[39] <= xn[38];
        xn[40] <= xn[39];
        xn[41] <= xn[40];
        xn[42] <= xn[41];
        xn[43] <= xn[42];
        xn[44] <= xn[43];
        xn[45] <= xn[44];
        xn[46] <= xn[45];
        xn[47] <= xn[46];
        xn[48] <= xn[47];
        xn[49] <= xn[48];
        xn[50] <= xn[49];
        xn[51] <= xn[50];
        xn[52] <= xn[51];
        xn[53] <= xn[52];
        xn[54] <= xn[53];
        xn[55] <= xn[54];
        xn[56] <= xn[55];
        xn[57] <= xn[56];
        xn[58] <= xn[57];
        xn[59] <= xn[58];
        xn[60] <= xn[59];
        xn[61] <= xn[60];
        xn[62] <= xn[61];
        xn[63] <= xn[62];
        xn[64] <= xn[63];
        xn[65] <= xn[64];
        xn[66] <= xn[65];
        xn[67] <= xn[66];
        xn[68] <= xn[67];
        xn[69] <= xn[68];
        xn[70] <= xn[69];
        xn[71] <= xn[70];
        xn[72] <= xn[71];
        xn[73] <= xn[72];
        xn[74] <= xn[73];
        xn[75] <= xn[74];
        xn[76] <= xn[75];
        xn[77] <= xn[76];
        xn[78] <= xn[77];
        xn[79] <= xn[78];
        xn[80] <= xn[79];
        xn[81] <= xn[80];
        xn[82] <= xn[81];
        xn[83] <= xn[82];
        xn[84] <= xn[83];
        xn[85] <= xn[84];
        xn[86] <= xn[85];
        xn[87] <= xn[86];
        xn[88] <= xn[87];
        xn[89] <= xn[88];
        xn[90] <= xn[89];
        xn[91] <= xn[90];
        xn[92] <= xn[91];
        xn[93] <= xn[92];
        xn[94] <= xn[93];
        xn[95] <= xn[94];
        xn[96] <= xn[95];
        xn[97] <= xn[96];
        xn[98] <= xn[97];
        xn[99] <= xn[98];
        xn[100] <= xn[99];
        xn[101] <= xn[100];
        xn[102] <= xn[101];
        xn[103] <= xn[102];
        xn[104] <= xn[103];
        xn[105] <= xn[104];
        xn[106] <= xn[105];
        xn[107] <= xn[106];
        xn[108] <= xn[107];
        xn[109] <= xn[108];
        xn[110] <= xn[109];
        xn[111] <= xn[110];
        xn[112] <= xn[111];
        xn[113] <= xn[112];
        xn[114] <= xn[113];
        xn[115] <= xn[114];
        xn[116] <= xn[115];
        xn[117] <= xn[116];
        xn[118] <= xn[117];
        xn[119] <= xn[118];
        xn[120] <= xn[119];
        xn[121] <= xn[120];
        xn[122] <= xn[121];
        xn[123] <= xn[122];
        xn[124] <= xn[123];
        xn[125] <= xn[124];
        xn[126] <= xn[125];
        xn[127] <= xn[126];
        xn[128] <= xn[127];
        xn[129] <= xn[128];
        xn[130] <= xn[129];
        xn[131] <= xn[130];
        xn[132] <= xn[131];
        xn[133] <= xn[132];
        xn[134] <= xn[133];
        xn[135] <= xn[134];
        xn[136] <= xn[135];
        xn[137] <= xn[136];
        xn[138] <= xn[137];
        xn[139] <= xn[138];
        xn[140] <= xn[139];
        xn[141] <= xn[140];
        xn[142] <= xn[141];
        xn[143] <= xn[142];
        xn[144] <= xn[143];
        xn[145] <= xn[144];
        xn[146] <= xn[145];
        xn[147] <= xn[146];
        xn[148] <= xn[147];
        xn[149] <= xn[148];
        xn[150] <= xn[149];
        xn[151] <= xn[150];
        xn[152] <= xn[151];
        xn[153] <= xn[152];
        xn[154] <= xn[153];
        xn[155] <= xn[154];
        xn[156] <= xn[155];
        xn[157] <= xn[156];
        xn[158] <= xn[157];
        xn[159] <= xn[158];
        xn[160] <= xn[159];
        xn[161] <= xn[160];
        xn[162] <= xn[161];
        xn[163] <= xn[162];
        xn[164] <= xn[163];
        xn[165] <= xn[164];
        xn[166] <= xn[165];
        xn[167] <= xn[166];
        xn[168] <= xn[167];
        xn[169] <= xn[168];
        xn[170] <= xn[169];
        xn[171] <= xn[170];
        xn[172] <= xn[171];
        xn[173] <= xn[172];
        xn[174] <= xn[173];
        xn[175] <= xn[174];
        xn[176] <= xn[175];
        xn[177] <= xn[176];
        xn[178] <= xn[177];
        xn[179] <= xn[178];
        xn[180] <= xn[179];
        xn[181] <= xn[180];
        xn[182] <= xn[181];
        xn[183] <= xn[182];
        xn[184] <= xn[183];
        xn[185] <= xn[184];
        xn[186] <= xn[185];
        xn[187] <= xn[186];
        xn[188] <= xn[187];
        xn[189] <= xn[188];
        xn[190] <= xn[189];
        xn[191] <= xn[190];
        xn[192] <= xn[191];
        xn[193] <= xn[192];
        xn[194] <= xn[193];
        xn[195] <= xn[194];
        xn[196] <= xn[195];
        xn[197] <= xn[196];
        xn[198] <= xn[197];
        xn[199] <= xn[198];
		end //end if(audio_done)
	end
	else begin
		accum_left <= accum_left + product_left;
		accum_right <= accum_right + product_right;
		counter <= counter+1;
		done <= 0;
	end
	if(first) begin
    case(counter)
		  8'd0 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd1 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd2 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd3 : begin  h_arr_left[counter] = -27'd3; h_arr_right[counter] = 27'd3; end
        8'd4 : begin  h_arr_left[counter] = -27'd11; h_arr_right[counter] = -27'd21; end
        8'd5 : begin  h_arr_left[counter] = -27'd27; h_arr_right[counter] = -27'd17; end
        8'd6 : begin  h_arr_left[counter] = -27'd31; h_arr_right[counter] = -27'd84; end
        8'd7 : begin  h_arr_left[counter] = -27'd124; h_arr_right[counter] = -27'd44; end
        8'd8 : begin  h_arr_left[counter] = -27'd144; h_arr_right[counter] = -27'd97; end
        8'd9 : begin  h_arr_left[counter] = -27'd77; h_arr_right[counter] = 27'd25; end
        8'd10 : begin  h_arr_left[counter] = -27'd245; h_arr_right[counter] = -27'd126; end
        8'd11 : begin  h_arr_left[counter] = -27'd256; h_arr_right[counter] = 27'd64; end
        8'd12 : begin  h_arr_left[counter] = -27'd199; h_arr_right[counter] = -27'd397; end
        8'd13 : begin  h_arr_left[counter] = -27'd207; h_arr_right[counter] = 27'd9; end
        8'd14 : begin  h_arr_left[counter] = -27'd221; h_arr_right[counter] = -27'd765; end
        8'd15 : begin  h_arr_left[counter] = -27'd326; h_arr_right[counter] = -27'd257; end
        8'd16 : begin  h_arr_left[counter] = -27'd262; h_arr_right[counter] = 27'd5553; end
        8'd17 : begin  h_arr_left[counter] = -27'd288; h_arr_right[counter] = 27'd1874; end
        8'd18 : begin  h_arr_left[counter] = -27'd303; h_arr_right[counter] = -27'd111; end
        8'd19 : begin  h_arr_left[counter] = -27'd312; h_arr_right[counter] = -27'd4930; end
        8'd20 : begin  h_arr_left[counter] = -27'd273; h_arr_right[counter] = -27'd2856; end
        8'd21 : begin  h_arr_left[counter] = -27'd277; h_arr_right[counter] = 27'd429; end
        8'd22 : begin  h_arr_left[counter] = -27'd286; h_arr_right[counter] = -27'd4971; end
        8'd23 : begin  h_arr_left[counter] = -27'd376; h_arr_right[counter] = 27'd14191; end
        8'd24 : begin  h_arr_left[counter] = -27'd410; h_arr_right[counter] = -27'd20656; end
        8'd25 : begin  h_arr_left[counter] = -27'd220; h_arr_right[counter] = 27'd101647; end
        8'd26 : begin  h_arr_left[counter] = -27'd364; h_arr_right[counter] = 27'd177467; end
        8'd27 : begin  h_arr_left[counter] = -27'd332; h_arr_right[counter] = 27'd37712; end
        8'd28 : begin  h_arr_left[counter] = -27'd345; h_arr_right[counter] = -27'd57973; end
        8'd29 : begin  h_arr_left[counter] = -27'd316; h_arr_right[counter] = -27'd106775; end
        8'd30 : begin  h_arr_left[counter] = -27'd322; h_arr_right[counter] = -27'd61845; end
        8'd31 : begin  h_arr_left[counter] = -27'd350; h_arr_right[counter] = -27'd96128; end
        8'd32 : begin  h_arr_left[counter] = -27'd262; h_arr_right[counter] = -27'd20200; end
        8'd33 : begin  h_arr_left[counter] = -27'd349; h_arr_right[counter] = 27'd113478; end
        8'd34 : begin  h_arr_left[counter] = -27'd496; h_arr_right[counter] = 27'd49487; end
        8'd35 : begin  h_arr_left[counter] = -27'd349; h_arr_right[counter] = -27'd33361; end
        8'd36 : begin  h_arr_left[counter] = 27'd80; h_arr_right[counter] = -27'd14884; end
        8'd37 : begin  h_arr_left[counter] = -27'd354; h_arr_right[counter] = -27'd14154; end
        8'd38 : begin  h_arr_left[counter] = -27'd513; h_arr_right[counter] = -27'd22352; end
        8'd39 : begin  h_arr_left[counter] = -27'd401; h_arr_right[counter] = -27'd17650; end
        8'd40 : begin  h_arr_left[counter] = -27'd262; h_arr_right[counter] = -27'd7044; end
        8'd41 : begin  h_arr_left[counter] = 27'd168; h_arr_right[counter] = 27'd4546; end
        8'd42 : begin  h_arr_left[counter] = -27'd333; h_arr_right[counter] = -27'd12513; end
        8'd43 : begin  h_arr_left[counter] = -27'd441; h_arr_right[counter] = -27'd14205; end
        8'd44 : begin  h_arr_left[counter] = -27'd491; h_arr_right[counter] = 27'd5399; end
        8'd45 : begin  h_arr_left[counter] = -27'd329; h_arr_right[counter] = -27'd7069; end
        8'd46 : begin  h_arr_left[counter] = -27'd565; h_arr_right[counter] = -27'd25698; end
        8'd47 : begin  h_arr_left[counter] = -27'd632; h_arr_right[counter] = 27'd1162; end
        8'd48 : begin  h_arr_left[counter] = -27'd528; h_arr_right[counter] = 27'd5326; end
        8'd49 : begin  h_arr_left[counter] = -27'd665; h_arr_right[counter] = -27'd9585; end
        8'd50 : begin  h_arr_left[counter] = -27'd711; h_arr_right[counter] = -27'd14721; end
        8'd51 : begin  h_arr_left[counter] = -27'd429; h_arr_right[counter] = -27'd7879; end
        8'd52 : begin  h_arr_left[counter] = 27'd644; h_arr_right[counter] = -27'd2181; end
        8'd53 : begin  h_arr_left[counter] = 27'd8254; h_arr_right[counter] = -27'd5182; end
        8'd54 : begin  h_arr_left[counter] = 27'd17995; h_arr_right[counter] = 27'd493; end
        8'd55 : begin  h_arr_left[counter] = 27'd16932; h_arr_right[counter] = 27'd9; end
        8'd56 : begin  h_arr_left[counter] = 27'd13874; h_arr_right[counter] = -27'd1124; end
        8'd57 : begin  h_arr_left[counter] = 27'd12025; h_arr_right[counter] = -27'd262; end
        8'd58 : begin  h_arr_left[counter] = 27'd5793; h_arr_right[counter] = 27'd2453; end
        8'd59 : begin  h_arr_left[counter] = -27'd1813; h_arr_right[counter] = -27'd1154; end
        8'd60 : begin  h_arr_left[counter] = -27'd6044; h_arr_right[counter] = -27'd8226; end
        8'd61 : begin  h_arr_left[counter] = -27'd6397; h_arr_right[counter] = -27'd4442; end
        8'd62 : begin  h_arr_left[counter] = -27'd4578; h_arr_right[counter] = 27'd7497; end
        8'd63 : begin  h_arr_left[counter] = -27'd461; h_arr_right[counter] = 27'd5851; end
        8'd64 : begin  h_arr_left[counter] = 27'd2741; h_arr_right[counter] = -27'd3831; end
        8'd65 : begin  h_arr_left[counter] = -27'd2211; h_arr_right[counter] = -27'd6957; end
        8'd66 : begin  h_arr_left[counter] = -27'd12430; h_arr_right[counter] = -27'd1538; end
        8'd67 : begin  h_arr_left[counter] = -27'd10603; h_arr_right[counter] = 27'd1204; end
        8'd68 : begin  h_arr_left[counter] = -27'd2668; h_arr_right[counter] = 27'd1346; end
        8'd69 : begin  h_arr_left[counter] = -27'd4701; h_arr_right[counter] = 27'd3561; end
        8'd70 : begin  h_arr_left[counter] = -27'd7540; h_arr_right[counter] = -27'd534; end
        8'd71 : begin  h_arr_left[counter] = -27'd4281; h_arr_right[counter] = -27'd2155; end
        8'd72 : begin  h_arr_left[counter] = -27'd2766; h_arr_right[counter] = -27'd506; end
        8'd73 : begin  h_arr_left[counter] = -27'd1797; h_arr_right[counter] = -27'd957; end
        8'd74 : begin  h_arr_left[counter] = -27'd1122; h_arr_right[counter] = -27'd2533; end
        8'd75 : begin  h_arr_left[counter] = -27'd3857; h_arr_right[counter] = 27'd151; end
        8'd76 : begin  h_arr_left[counter] = -27'd7132; h_arr_right[counter] = 27'd6358; end
        8'd77 : begin  h_arr_left[counter] = -27'd6628; h_arr_right[counter] = 27'd8828; end
        8'd78 : begin  h_arr_left[counter] = -27'd4707; h_arr_right[counter] = 27'd3400; end
        8'd79 : begin  h_arr_left[counter] = -27'd4390; h_arr_right[counter] = -27'd1112; end
        8'd80 : begin  h_arr_left[counter] = -27'd5020; h_arr_right[counter] = 27'd945; end
        8'd81 : begin  h_arr_left[counter] = -27'd6130; h_arr_right[counter] = 27'd1338; end
        8'd82 : begin  h_arr_left[counter] = -27'd5199; h_arr_right[counter] = 27'd1349; end
        8'd83 : begin  h_arr_left[counter] = -27'd3167; h_arr_right[counter] = 27'd1759; end
        8'd84 : begin  h_arr_left[counter] = -27'd2770; h_arr_right[counter] = 27'd1976; end
        8'd85 : begin  h_arr_left[counter] = -27'd3291; h_arr_right[counter] = 27'd733; end
        8'd86 : begin  h_arr_left[counter] = -27'd2960; h_arr_right[counter] = 27'd829; end
        8'd87 : begin  h_arr_left[counter] = -27'd2307; h_arr_right[counter] = 27'd2155; end
        8'd88 : begin  h_arr_left[counter] = -27'd2538; h_arr_right[counter] = 27'd687; end
        8'd89 : begin  h_arr_left[counter] = -27'd2961; h_arr_right[counter] = -27'd1668; end
        8'd90 : begin  h_arr_left[counter] = -27'd2862; h_arr_right[counter] = -27'd1693; end
        8'd91 : begin  h_arr_left[counter] = -27'd3370; h_arr_right[counter] = 27'd495; end
        8'd92 : begin  h_arr_left[counter] = -27'd2761; h_arr_right[counter] = 27'd1658; end
        8'd93 : begin  h_arr_left[counter] = -27'd296; h_arr_right[counter] = 27'd2976; end
        8'd94 : begin  h_arr_left[counter] = 27'd591; h_arr_right[counter] = 27'd2907; end
        8'd95 : begin  h_arr_left[counter] = 27'd466; h_arr_right[counter] = 27'd1580; end
        8'd96 : begin  h_arr_left[counter] = 27'd122; h_arr_right[counter] = 27'd1198; end
        8'd97 : begin  h_arr_left[counter] = -27'd52; h_arr_right[counter] = 27'd554; end
        8'd98 : begin  h_arr_left[counter] = 27'd3; h_arr_right[counter] = -27'd143; end
        8'd99 : begin  h_arr_left[counter] = -27'd479; h_arr_right[counter] = -27'd21; end
        8'd100 : begin  h_arr_left[counter] = -27'd1169; h_arr_right[counter] = 27'd1437; end
        8'd101 : begin  h_arr_left[counter] = -27'd1672; h_arr_right[counter] = 27'd2442; end
        8'd102 : begin  h_arr_left[counter] = -27'd1639; h_arr_right[counter] = 27'd807; end
        8'd103 : begin  h_arr_left[counter] = -27'd1456; h_arr_right[counter] = -27'd223; end
        8'd104 : begin  h_arr_left[counter] = -27'd1439; h_arr_right[counter] = 27'd614; end
        8'd105 : begin  h_arr_left[counter] = -27'd1117; h_arr_right[counter] = 27'd1344; end
        8'd106 : begin  h_arr_left[counter] = -27'd793; h_arr_right[counter] = 27'd1170; end
        8'd107 : begin  h_arr_left[counter] = -27'd625; h_arr_right[counter] = 27'd339; end
        8'd108 : begin  h_arr_left[counter] = -27'd82; h_arr_right[counter] = 27'd514; end
        8'd109 : begin  h_arr_left[counter] = 27'd44; h_arr_right[counter] = 27'd1664; end
        8'd110 : begin  h_arr_left[counter] = 27'd11; h_arr_right[counter] = 27'd1752; end
        8'd111 : begin  h_arr_left[counter] = 27'd265; h_arr_right[counter] = 27'd1842; end
        8'd112 : begin  h_arr_left[counter] = 27'd496; h_arr_right[counter] = 27'd2074; end
        8'd113 : begin  h_arr_left[counter] = 27'd581; h_arr_right[counter] = 27'd963; end
        8'd114 : begin  h_arr_left[counter] = 27'd341; h_arr_right[counter] = 27'd663; end
        8'd115 : begin  h_arr_left[counter] = 27'd86; h_arr_right[counter] = 27'd1336; end
        8'd116 : begin  h_arr_left[counter] = -27'd71; h_arr_right[counter] = 27'd1479; end
        8'd117 : begin  h_arr_left[counter] = -27'd203; h_arr_right[counter] = 27'd989; end
        8'd118 : begin  h_arr_left[counter] = -27'd72; h_arr_right[counter] = 27'd613; end
        8'd119 : begin  h_arr_left[counter] = 27'd137; h_arr_right[counter] = 27'd711; end
        8'd120 : begin  h_arr_left[counter] = 27'd335; h_arr_right[counter] = 27'd172; end
        8'd121 : begin  h_arr_left[counter] = 27'd757; h_arr_right[counter] = -27'd528; end
        8'd122 : begin  h_arr_left[counter] = 27'd1868; h_arr_right[counter] = -27'd547; end
        8'd123 : begin  h_arr_left[counter] = 27'd1951; h_arr_right[counter] = 27'd410; end
        8'd124 : begin  h_arr_left[counter] = 27'd1028; h_arr_right[counter] = 27'd958; end
        8'd125 : begin  h_arr_left[counter] = 27'd616; h_arr_right[counter] = 27'd133; end
        8'd126 : begin  h_arr_left[counter] = -27'd183; h_arr_right[counter] = -27'd341; end
        8'd127 : begin  h_arr_left[counter] = -27'd951; h_arr_right[counter] = -27'd535; end
        8'd128 : begin  h_arr_left[counter] = -27'd1092; h_arr_right[counter] = -27'd1370; end
        8'd129 : begin  h_arr_left[counter] = -27'd1101; h_arr_right[counter] = -27'd1054; end
        8'd130 : begin  h_arr_left[counter] = -27'd526; h_arr_right[counter] = 27'd249; end
        8'd131 : begin  h_arr_left[counter] = 27'd185; h_arr_right[counter] = 27'd690; end
        8'd132 : begin  h_arr_left[counter] = 27'd1115; h_arr_right[counter] = 27'd519; end
        8'd133 : begin  h_arr_left[counter] = 27'd1528; h_arr_right[counter] = 27'd33; end
        8'd134 : begin  h_arr_left[counter] = 27'd1483; h_arr_right[counter] = 27'd13; end
        8'd135 : begin  h_arr_left[counter] = 27'd1643; h_arr_right[counter] = 27'd346; end
        8'd136 : begin  h_arr_left[counter] = 27'd1286; h_arr_right[counter] = 27'd382; end
        8'd137 : begin  h_arr_left[counter] = 27'd151; h_arr_right[counter] = -27'd395; end
        8'd138 : begin  h_arr_left[counter] = -27'd805; h_arr_right[counter] = -27'd948; end
        8'd139 : begin  h_arr_left[counter] = -27'd1182; h_arr_right[counter] = -27'd8; end
        8'd140 : begin  h_arr_left[counter] = -27'd1168; h_arr_right[counter] = 27'd304; end
        8'd141 : begin  h_arr_left[counter] = -27'd601; h_arr_right[counter] = 27'd789; end
        8'd142 : begin  h_arr_left[counter] = -27'd303; h_arr_right[counter] = 27'd1582; end
        8'd143 : begin  h_arr_left[counter] = -27'd444; h_arr_right[counter] = 27'd1495; end
        8'd144 : begin  h_arr_left[counter] = -27'd201; h_arr_right[counter] = 27'd803; end
        8'd145 : begin  h_arr_left[counter] = 27'd316; h_arr_right[counter] = -27'd34; end
        8'd146 : begin  h_arr_left[counter] = 27'd194; h_arr_right[counter] = -27'd518; end
        8'd147 : begin  h_arr_left[counter] = 27'd46; h_arr_right[counter] = -27'd776; end
        8'd148 : begin  h_arr_left[counter] = 27'd33; h_arr_right[counter] = 27'd135; end
        8'd149 : begin  h_arr_left[counter] = -27'd147; h_arr_right[counter] = 27'd1558; end
        8'd150 : begin  h_arr_left[counter] = -27'd27; h_arr_right[counter] = 27'd1607; end
        8'd151 : begin  h_arr_left[counter] = -27'd183; h_arr_right[counter] = 27'd1482; end
        8'd152 : begin  h_arr_left[counter] = -27'd336; h_arr_right[counter] = 27'd2084; end
        8'd153 : begin  h_arr_left[counter] = -27'd384; h_arr_right[counter] = 27'd1597; end
        8'd154 : begin  h_arr_left[counter] = -27'd222; h_arr_right[counter] = 27'd641; end
        8'd155 : begin  h_arr_left[counter] = 27'd122; h_arr_right[counter] = 27'd177; end
        8'd156 : begin  h_arr_left[counter] = 27'd568; h_arr_right[counter] = -27'd268; end
        8'd157 : begin  h_arr_left[counter] = 27'd799; h_arr_right[counter] = 27'd330; end
        8'd158 : begin  h_arr_left[counter] = 27'd1005; h_arr_right[counter] = 27'd654; end
        8'd159 : begin  h_arr_left[counter] = 27'd951; h_arr_right[counter] = 27'd613; end
        8'd160 : begin  h_arr_left[counter] = 27'd152; h_arr_right[counter] = 27'd883; end
        8'd161 : begin  h_arr_left[counter] = 27'd43; h_arr_right[counter] = 27'd260; end
        8'd162 : begin  h_arr_left[counter] = 27'd189; h_arr_right[counter] = -27'd315; end
        8'd163 : begin  h_arr_left[counter] = -27'd40; h_arr_right[counter] = -27'd157; end
        8'd164 : begin  h_arr_left[counter] = 27'd136; h_arr_right[counter] = 27'd7; end
        8'd165 : begin  h_arr_left[counter] = 27'd437; h_arr_right[counter] = -27'd300; end
        8'd166 : begin  h_arr_left[counter] = 27'd617; h_arr_right[counter] = -27'd507; end
        8'd167 : begin  h_arr_left[counter] = 27'd762; h_arr_right[counter] = -27'd156; end
        8'd168 : begin  h_arr_left[counter] = 27'd786; h_arr_right[counter] = -27'd115; end
        8'd169 : begin  h_arr_left[counter] = 27'd728; h_arr_right[counter] = -27'd191; end
        8'd170 : begin  h_arr_left[counter] = 27'd537; h_arr_right[counter] = 27'd264; end
        8'd171 : begin  h_arr_left[counter] = 27'd460; h_arr_right[counter] = 27'd488; end
        8'd172 : begin  h_arr_left[counter] = 27'd453; h_arr_right[counter] = 27'd290; end
        8'd173 : begin  h_arr_left[counter] = 27'd254; h_arr_right[counter] = 27'd236; end
        8'd174 : begin  h_arr_left[counter] = 27'd172; h_arr_right[counter] = 27'd191; end
        8'd175 : begin  h_arr_left[counter] = 27'd254; h_arr_right[counter] = 27'd183; end
        8'd176 : begin  h_arr_left[counter] = 27'd261; h_arr_right[counter] = -27'd24; end
        8'd177 : begin  h_arr_left[counter] = 27'd194; h_arr_right[counter] = -27'd227; end
        8'd178 : begin  h_arr_left[counter] = 27'd161; h_arr_right[counter] = -27'd53; end
        8'd179 : begin  h_arr_left[counter] = 27'd153; h_arr_right[counter] = 27'd64; end
        8'd180 : begin  h_arr_left[counter] = 27'd178; h_arr_right[counter] = 27'd12; end
        8'd181 : begin  h_arr_left[counter] = 27'd153; h_arr_right[counter] = -27'd94; end
        8'd182 : begin  h_arr_left[counter] = 27'd121; h_arr_right[counter] = -27'd205; end
        8'd183 : begin  h_arr_left[counter] = 27'd139; h_arr_right[counter] = -27'd167; end
        8'd184 : begin  h_arr_left[counter] = 27'd116; h_arr_right[counter] = 27'd109; end
        8'd185 : begin  h_arr_left[counter] = 27'd72; h_arr_right[counter] = 27'd282; end
        8'd186 : begin  h_arr_left[counter] = 27'd55; h_arr_right[counter] = 27'd232; end
        8'd187 : begin  h_arr_left[counter] = 27'd58; h_arr_right[counter] = 27'd174; end
        8'd188 : begin  h_arr_left[counter] = 27'd52; h_arr_right[counter] = 27'd95; end
        8'd189 : begin  h_arr_left[counter] = 27'd45; h_arr_right[counter] = -27'd41; end
        8'd190 : begin  h_arr_left[counter] = 27'd39; h_arr_right[counter] = -27'd130; end
        8'd191 : begin  h_arr_left[counter] = 27'd40; h_arr_right[counter] = -27'd85; end
        8'd192 : begin  h_arr_left[counter] = 27'd42; h_arr_right[counter] = -27'd7; end
        8'd193 : begin  h_arr_left[counter] = 27'd37; h_arr_right[counter] = -27'd1; end
        8'd194 : begin  h_arr_left[counter] = 27'd31; h_arr_right[counter] = -27'd5; end
        8'd195 : begin  h_arr_left[counter] = 27'd26; h_arr_right[counter] = 27'd1; end
        8'd196 : begin  h_arr_left[counter] = 27'd20; h_arr_right[counter] = -27'd1; end
        8'd197 : begin  h_arr_left[counter] = 27'd12; h_arr_right[counter] = -27'd4; end
        8'd198 : begin  h_arr_left[counter] = 27'd7; h_arr_right[counter] = -27'd7; end
        8'd199 : begin  h_arr_left[counter] = 27'd3; h_arr_right[counter] = -27'd3; first = 0; end
    endcase
    end
        
    end
end
endmodule

module FIR_filter_f(x, clk, reset, yn_left, yn_right, done, audio_done);
input signed [26:0] x;
input clk;
input reset;
input audio_done;
output reg signed [26:0] yn_left;
output reg signed [26:0] yn_right;
output reg done;

reg signed[26:0] xn [199:0];
wire signed [26:0] v;
reg signed [26:0] h_arr_left [199:0];
reg signed [26:0] h_arr_right [199:0];
wire signed [26:0] product_left;
reg signed [26:0] accum_left;
wire signed [26:0] product_right;
reg signed [26:0] accum_right;
reg [7:0] counter;
reg first;


signed_mult_27 multl(product_left, xn[counter], h_arr_left[counter]);
signed_mult_27 multr(product_right, xn[counter], h_arr_right[counter]);

//assign v = ( intermediate[0] + intermediate[1] + intermediate[2] + intermediate[3] + intermediate[4] + intermediate[5] + intermediate[6] + intermediate[7] + intermediate[8] + intermediate[9] + intermediate[10] + intermediate[11] + intermediate[12] + intermediate[13] + intermediate[14] + intermediate[15] + intermediate[16] + intermediate[17] + intermediate[18] + intermediate[19] + intermediate[20] + intermediate[21] + intermediate[22] + intermediate[23] + intermediate[24] + intermediate[25] + intermediate[26] + intermediate[27] + intermediate[28] + intermediate[29] + intermediate[30] + intermediate[31] + intermediate[32] + intermediate[33] + intermediate[34] + intermediate[35] + intermediate[36] + intermediate[37] + intermediate[38] + intermediate[39] + intermediate[40] + intermediate[41] + intermediate[42] + intermediate[43] + intermediate[44] + intermediate[45] + intermediate[46] + intermediate[47] + intermediate[48] + intermediate[49] + intermediate[50] + intermediate[51] + intermediate[52] + intermediate[53] + intermediate[54] + intermediate[55] + intermediate[56] + intermediate[57] + intermediate[58] + intermediate[59] + intermediate[60] + intermediate[61] + intermediate[62] + intermediate[63] + intermediate[64] + intermediate[65] + intermediate[66] + intermediate[67] + intermediate[68] + intermediate[69] + intermediate[70] + intermediate[71] + intermediate[72] + intermediate[73] + intermediate[74] + intermediate[75] + intermediate[76] + intermediate[77] + intermediate[78] + intermediate[79] + intermediate[80] + intermediate[81] + intermediate[82] + intermediate[83] + intermediate[84] + intermediate[85] + intermediate[86] + intermediate[87] + intermediate[88] + intermediate[89] + intermediate[90] + intermediate[91] + intermediate[92] + intermediate[93] + intermediate[94] + intermediate[95] + intermediate[96] + intermediate[97] + intermediate[98] + intermediate[99] + intermediate[100] + intermediate[101] + intermediate[102] + intermediate[103] + intermediate[104] + intermediate[105] + intermediate[106] + intermediate[107] + intermediate[108] + intermediate[109] + intermediate[110] + intermediate[111] + intermediate[112] + intermediate[113] + intermediate[114] + intermediate[115] + intermediate[116] + intermediate[117] + intermediate[118] + intermediate[119] + intermediate[120] + intermediate[121] + intermediate[122] + intermediate[123] + intermediate[124] + intermediate[125] + intermediate[126] + intermediate[127] + intermediate[128] + intermediate[129] + intermediate[130] + intermediate[131] + intermediate[132] + intermediate[133] + intermediate[134] + intermediate[135] + intermediate[136] + intermediate[137] + intermediate[138] + intermediate[139] + intermediate[140] + intermediate[141] + intermediate[142] + intermediate[143] + intermediate[144] + intermediate[145] + intermediate[146] + intermediate[147] + intermediate[148] + intermediate[149] + intermediate[150] + intermediate[151] + intermediate[152] + intermediate[153] + intermediate[154] + intermediate[155] + intermediate[156] + intermediate[157] + intermediate[158] + intermediate[159] + intermediate[160] + intermediate[161] + intermediate[162] + intermediate[163] + intermediate[164] + intermediate[165] + intermediate[166] + intermediate[167] + intermediate[168] + intermediate[169] + intermediate[170] + intermediate[171] + intermediate[172] + intermediate[173] + intermediate[174] + intermediate[175] + intermediate[176] + intermediate[177] + intermediate[178] + intermediate[179] + intermediate[180] + intermediate[181] + intermediate[182] + intermediate[183] + intermediate[184] + intermediate[185] + intermediate[186] + intermediate[187] + intermediate[188] + intermediate[189] + intermediate[190] + intermediate[191] + intermediate[192] + intermediate[193] + intermediate[194] + intermediate[195] + intermediate[196] + intermediate[197] + intermediate[198] + intermediate[199]);
always @ (posedge clk) begin
    if(reset) begin
	counter <= 8'd0;
	accum_left <= 8'd0;
	accum_right <= 8'd0;
	done <= 0;
	first <= 1;
        xn[0] <= 27'd0;
        xn[1] <= 27'd0;
        xn[2] <= 27'd0;
        xn[3] <= 27'd0;
        xn[4] <= 27'd0;
        xn[5] <= 27'd0;
        xn[6] <= 27'd0;
        xn[7] <= 27'd0;
        xn[8] <= 27'd0;
        xn[9] <= 27'd0;
        xn[10] <= 27'd0;
        xn[11] <= 27'd0;
        xn[12] <= 27'd0;
        xn[13] <= 27'd0;
        xn[14] <= 27'd0;
        xn[15] <= 27'd0;
        xn[16] <= 27'd0;
        xn[17] <= 27'd0;
        xn[18] <= 27'd0;
        xn[19] <= 27'd0;
        xn[20] <= 27'd0;
        xn[21] <= 27'd0;
        xn[22] <= 27'd0;
        xn[23] <= 27'd0;
        xn[24] <= 27'd0;
        xn[25] <= 27'd0;
        xn[26] <= 27'd0;
        xn[27] <= 27'd0;
        xn[28] <= 27'd0;
        xn[29] <= 27'd0;
        xn[30] <= 27'd0;
        xn[31] <= 27'd0;
        xn[32] <= 27'd0;
        xn[33] <= 27'd0;
        xn[34] <= 27'd0;
        xn[35] <= 27'd0;
        xn[36] <= 27'd0;
        xn[37] <= 27'd0;
        xn[38] <= 27'd0;
        xn[39] <= 27'd0;
        xn[40] <= 27'd0;
        xn[41] <= 27'd0;
        xn[42] <= 27'd0;
        xn[43] <= 27'd0;
        xn[44] <= 27'd0;
        xn[45] <= 27'd0;
        xn[46] <= 27'd0;
        xn[47] <= 27'd0;
        xn[48] <= 27'd0;
        xn[49] <= 27'd0;
        xn[50] <= 27'd0;
        xn[51] <= 27'd0;
        xn[52] <= 27'd0;
        xn[53] <= 27'd0;
        xn[54] <= 27'd0;
        xn[55] <= 27'd0;
        xn[56] <= 27'd0;
        xn[57] <= 27'd0;
        xn[58] <= 27'd0;
        xn[59] <= 27'd0;
        xn[60] <= 27'd0;
        xn[61] <= 27'd0;
        xn[62] <= 27'd0;
        xn[63] <= 27'd0;
        xn[64] <= 27'd0;
        xn[65] <= 27'd0;
        xn[66] <= 27'd0;
        xn[67] <= 27'd0;
        xn[68] <= 27'd0;
        xn[69] <= 27'd0;
        xn[70] <= 27'd0;
        xn[71] <= 27'd0;
        xn[72] <= 27'd0;
        xn[73] <= 27'd0;
        xn[74] <= 27'd0;
        xn[75] <= 27'd0;
        xn[76] <= 27'd0;
        xn[77] <= 27'd0;
        xn[78] <= 27'd0;
        xn[79] <= 27'd0;
        xn[80] <= 27'd0;
        xn[81] <= 27'd0;
        xn[82] <= 27'd0;
        xn[83] <= 27'd0;
        xn[84] <= 27'd0;
        xn[85] <= 27'd0;
        xn[86] <= 27'd0;
        xn[87] <= 27'd0;
        xn[88] <= 27'd0;
        xn[89] <= 27'd0;
        xn[90] <= 27'd0;
        xn[91] <= 27'd0;
        xn[92] <= 27'd0;
        xn[93] <= 27'd0;
        xn[94] <= 27'd0;
        xn[95] <= 27'd0;
        xn[96] <= 27'd0;
        xn[97] <= 27'd0;
        xn[98] <= 27'd0;
        xn[99] <= 27'd0;
        xn[100] <= 27'd0;
        xn[101] <= 27'd0;
        xn[102] <= 27'd0;
        xn[103] <= 27'd0;
        xn[104] <= 27'd0;
        xn[105] <= 27'd0;
        xn[106] <= 27'd0;
        xn[107] <= 27'd0;
        xn[108] <= 27'd0;
        xn[109] <= 27'd0;
        xn[110] <= 27'd0;
        xn[111] <= 27'd0;
        xn[112] <= 27'd0;
        xn[113] <= 27'd0;
        xn[114] <= 27'd0;
        xn[115] <= 27'd0;
        xn[116] <= 27'd0;
        xn[117] <= 27'd0;
        xn[118] <= 27'd0;
        xn[119] <= 27'd0;
        xn[120] <= 27'd0;
        xn[121] <= 27'd0;
        xn[122] <= 27'd0;
        xn[123] <= 27'd0;
        xn[124] <= 27'd0;
        xn[125] <= 27'd0;
        xn[126] <= 27'd0;
        xn[127] <= 27'd0;
        xn[128] <= 27'd0;
        xn[129] <= 27'd0;
        xn[130] <= 27'd0;
        xn[131] <= 27'd0;
        xn[132] <= 27'd0;
        xn[133] <= 27'd0;
        xn[134] <= 27'd0;
        xn[135] <= 27'd0;
        xn[136] <= 27'd0;
        xn[137] <= 27'd0;
        xn[138] <= 27'd0;
        xn[139] <= 27'd0;
        xn[140] <= 27'd0;
        xn[141] <= 27'd0;
        xn[142] <= 27'd0;
        xn[143] <= 27'd0;
        xn[144] <= 27'd0;
        xn[145] <= 27'd0;
        xn[146] <= 27'd0;
        xn[147] <= 27'd0;
        xn[148] <= 27'd0;
        xn[149] <= 27'd0;
        xn[150] <= 27'd0;
        xn[151] <= 27'd0;
        xn[152] <= 27'd0;
        xn[153] <= 27'd0;
        xn[154] <= 27'd0;
        xn[155] <= 27'd0;
        xn[156] <= 27'd0;
        xn[157] <= 27'd0;
        xn[158] <= 27'd0;
        xn[159] <= 27'd0;
        xn[160] <= 27'd0;
        xn[161] <= 27'd0;
        xn[162] <= 27'd0;
        xn[163] <= 27'd0;
        xn[164] <= 27'd0;
        xn[165] <= 27'd0;
        xn[166] <= 27'd0;
        xn[167] <= 27'd0;
        xn[168] <= 27'd0;
        xn[169] <= 27'd0;
        xn[170] <= 27'd0;
        xn[171] <= 27'd0;
        xn[172] <= 27'd0;
        xn[173] <= 27'd0;
        xn[174] <= 27'd0;
        xn[175] <= 27'd0;
        xn[176] <= 27'd0;
        xn[177] <= 27'd0;
        xn[178] <= 27'd0;
        xn[179] <= 27'd0;
        xn[180] <= 27'd0;
        xn[181] <= 27'd0;
        xn[182] <= 27'd0;
        xn[183] <= 27'd0;
        xn[184] <= 27'd0;
        xn[185] <= 27'd0;
        xn[186] <= 27'd0;
        xn[187] <= 27'd0;
        xn[188] <= 27'd0;
        xn[189] <= 27'd0;
        xn[190] <= 27'd0;
        xn[191] <= 27'd0;
        xn[192] <= 27'd0;
        xn[193] <= 27'd0;
        xn[194] <= 27'd0;
        xn[195] <= 27'd0;
        xn[196] <= 27'd0;
        xn[197] <= 27'd0;
        xn[198] <= 27'd0;
        xn[199] <= 27'd0;
        yn_left <= 27'd0;
		  yn_right <= 27'd0;
    end
    else begin
	if(counter == 8'd199) begin
		
		if (audio_done) begin
		done <= 1;
		yn_left <= accum_left;
		accum_left <= 8'd0;
		yn_right <= accum_right;
		accum_right <= 8'd0;	
		counter<=8'd0;
		xn[0] <= x;
        xn[1] <= xn[0];
        xn[2] <= xn[1];
        xn[3] <= xn[2];
        xn[4] <= xn[3];
        xn[5] <= xn[4];
        xn[6] <= xn[5];
        xn[7] <= xn[6];
        xn[8] <= xn[7];
        xn[9] <= xn[8];
        xn[10] <= xn[9];
        xn[11] <= xn[10];
        xn[12] <= xn[11];
        xn[13] <= xn[12];
        xn[14] <= xn[13];
        xn[15] <= xn[14];
        xn[16] <= xn[15];
        xn[17] <= xn[16];
        xn[18] <= xn[17];
        xn[19] <= xn[18];
        xn[20] <= xn[19];
        xn[21] <= xn[20];
        xn[22] <= xn[21];
        xn[23] <= xn[22];
        xn[24] <= xn[23];
        xn[25] <= xn[24];
        xn[26] <= xn[25];
        xn[27] <= xn[26];
        xn[28] <= xn[27];
        xn[29] <= xn[28];
        xn[30] <= xn[29];
        xn[31] <= xn[30];
        xn[32] <= xn[31];
        xn[33] <= xn[32];
        xn[34] <= xn[33];
        xn[35] <= xn[34];
        xn[36] <= xn[35];
        xn[37] <= xn[36];
        xn[38] <= xn[37];
        xn[39] <= xn[38];
        xn[40] <= xn[39];
        xn[41] <= xn[40];
        xn[42] <= xn[41];
        xn[43] <= xn[42];
        xn[44] <= xn[43];
        xn[45] <= xn[44];
        xn[46] <= xn[45];
        xn[47] <= xn[46];
        xn[48] <= xn[47];
        xn[49] <= xn[48];
        xn[50] <= xn[49];
        xn[51] <= xn[50];
        xn[52] <= xn[51];
        xn[53] <= xn[52];
        xn[54] <= xn[53];
        xn[55] <= xn[54];
        xn[56] <= xn[55];
        xn[57] <= xn[56];
        xn[58] <= xn[57];
        xn[59] <= xn[58];
        xn[60] <= xn[59];
        xn[61] <= xn[60];
        xn[62] <= xn[61];
        xn[63] <= xn[62];
        xn[64] <= xn[63];
        xn[65] <= xn[64];
        xn[66] <= xn[65];
        xn[67] <= xn[66];
        xn[68] <= xn[67];
        xn[69] <= xn[68];
        xn[70] <= xn[69];
        xn[71] <= xn[70];
        xn[72] <= xn[71];
        xn[73] <= xn[72];
        xn[74] <= xn[73];
        xn[75] <= xn[74];
        xn[76] <= xn[75];
        xn[77] <= xn[76];
        xn[78] <= xn[77];
        xn[79] <= xn[78];
        xn[80] <= xn[79];
        xn[81] <= xn[80];
        xn[82] <= xn[81];
        xn[83] <= xn[82];
        xn[84] <= xn[83];
        xn[85] <= xn[84];
        xn[86] <= xn[85];
        xn[87] <= xn[86];
        xn[88] <= xn[87];
        xn[89] <= xn[88];
        xn[90] <= xn[89];
        xn[91] <= xn[90];
        xn[92] <= xn[91];
        xn[93] <= xn[92];
        xn[94] <= xn[93];
        xn[95] <= xn[94];
        xn[96] <= xn[95];
        xn[97] <= xn[96];
        xn[98] <= xn[97];
        xn[99] <= xn[98];
        xn[100] <= xn[99];
        xn[101] <= xn[100];
        xn[102] <= xn[101];
        xn[103] <= xn[102];
        xn[104] <= xn[103];
        xn[105] <= xn[104];
        xn[106] <= xn[105];
        xn[107] <= xn[106];
        xn[108] <= xn[107];
        xn[109] <= xn[108];
        xn[110] <= xn[109];
        xn[111] <= xn[110];
        xn[112] <= xn[111];
        xn[113] <= xn[112];
        xn[114] <= xn[113];
        xn[115] <= xn[114];
        xn[116] <= xn[115];
        xn[117] <= xn[116];
        xn[118] <= xn[117];
        xn[119] <= xn[118];
        xn[120] <= xn[119];
        xn[121] <= xn[120];
        xn[122] <= xn[121];
        xn[123] <= xn[122];
        xn[124] <= xn[123];
        xn[125] <= xn[124];
        xn[126] <= xn[125];
        xn[127] <= xn[126];
        xn[128] <= xn[127];
        xn[129] <= xn[128];
        xn[130] <= xn[129];
        xn[131] <= xn[130];
        xn[132] <= xn[131];
        xn[133] <= xn[132];
        xn[134] <= xn[133];
        xn[135] <= xn[134];
        xn[136] <= xn[135];
        xn[137] <= xn[136];
        xn[138] <= xn[137];
        xn[139] <= xn[138];
        xn[140] <= xn[139];
        xn[141] <= xn[140];
        xn[142] <= xn[141];
        xn[143] <= xn[142];
        xn[144] <= xn[143];
        xn[145] <= xn[144];
        xn[146] <= xn[145];
        xn[147] <= xn[146];
        xn[148] <= xn[147];
        xn[149] <= xn[148];
        xn[150] <= xn[149];
        xn[151] <= xn[150];
        xn[152] <= xn[151];
        xn[153] <= xn[152];
        xn[154] <= xn[153];
        xn[155] <= xn[154];
        xn[156] <= xn[155];
        xn[157] <= xn[156];
        xn[158] <= xn[157];
        xn[159] <= xn[158];
        xn[160] <= xn[159];
        xn[161] <= xn[160];
        xn[162] <= xn[161];
        xn[163] <= xn[162];
        xn[164] <= xn[163];
        xn[165] <= xn[164];
        xn[166] <= xn[165];
        xn[167] <= xn[166];
        xn[168] <= xn[167];
        xn[169] <= xn[168];
        xn[170] <= xn[169];
        xn[171] <= xn[170];
        xn[172] <= xn[171];
        xn[173] <= xn[172];
        xn[174] <= xn[173];
        xn[175] <= xn[174];
        xn[176] <= xn[175];
        xn[177] <= xn[176];
        xn[178] <= xn[177];
        xn[179] <= xn[178];
        xn[180] <= xn[179];
        xn[181] <= xn[180];
        xn[182] <= xn[181];
        xn[183] <= xn[182];
        xn[184] <= xn[183];
        xn[185] <= xn[184];
        xn[186] <= xn[185];
        xn[187] <= xn[186];
        xn[188] <= xn[187];
        xn[189] <= xn[188];
        xn[190] <= xn[189];
        xn[191] <= xn[190];
        xn[192] <= xn[191];
        xn[193] <= xn[192];
        xn[194] <= xn[193];
        xn[195] <= xn[194];
        xn[196] <= xn[195];
        xn[197] <= xn[196];
        xn[198] <= xn[197];
        xn[199] <= xn[198];
		end //end if(audio_done)
	end
	else begin
		accum_left <= accum_left + product_left;
		accum_right <= accum_right + product_right;
		counter <= counter+1;
		done <= 0;
	end
	if(first) begin
    case(counter)
		   8'd0 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd1 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd2 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd3 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd4 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd5 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd6 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd7 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd8 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd9 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd10 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd11 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd12 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd13 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd14 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd15 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd16 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd17 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd18 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd19 : begin  h_arr_left[counter] = 27'd0; h_arr_right[counter] = 27'd0; end
        8'd20 : begin  h_arr_left[counter] = -27'd2; h_arr_right[counter] = -27'd4; end
        8'd21 : begin  h_arr_left[counter] = -27'd14; h_arr_right[counter] = -27'd6; end
        8'd22 : begin  h_arr_left[counter] = -27'd11; h_arr_right[counter] = -27'd5; end
        8'd23 : begin  h_arr_left[counter] = -27'd75; h_arr_right[counter] = -27'd26; end
        8'd24 : begin  h_arr_left[counter] = -27'd36; h_arr_right[counter] = -27'd51; end
        8'd25 : begin  h_arr_left[counter] = -27'd155; h_arr_right[counter] = 27'd72; end
        8'd26 : begin  h_arr_left[counter] = -27'd15; h_arr_right[counter] = -27'd232; end
        8'd27 : begin  h_arr_left[counter] = -27'd382; h_arr_right[counter] = -27'd52; end
        8'd28 : begin  h_arr_left[counter] = 27'd4; h_arr_right[counter] = -27'd360; end
        8'd29 : begin  h_arr_left[counter] = -27'd704; h_arr_right[counter] = 27'd305; end
        8'd30 : begin  h_arr_left[counter] = 27'd2955; h_arr_right[counter] = -27'd613; end
        8'd31 : begin  h_arr_left[counter] = 27'd1242; h_arr_right[counter] = 27'd2816; end
        8'd32 : begin  h_arr_left[counter] = -27'd4099; h_arr_right[counter] = 27'd4451; end
        8'd33 : begin  h_arr_left[counter] = 27'd607; h_arr_right[counter] = -27'd5355; end
        8'd34 : begin  h_arr_left[counter] = -27'd348; h_arr_right[counter] = -27'd2419; end
        8'd35 : begin  h_arr_left[counter] = -27'd567; h_arr_right[counter] = 27'd518; end
        8'd36 : begin  h_arr_left[counter] = -27'd606; h_arr_right[counter] = -27'd832; end
        8'd37 : begin  h_arr_left[counter] = 27'd430; h_arr_right[counter] = 27'd7; end
        8'd38 : begin  h_arr_left[counter] = -27'd1129; h_arr_right[counter] = 27'd1320; end
        8'd39 : begin  h_arr_left[counter] = 27'd516; h_arr_right[counter] = -27'd536; end
        8'd40 : begin  h_arr_left[counter] = 27'd3131; h_arr_right[counter] = 27'd5278; end
        8'd41 : begin  h_arr_left[counter] = 27'd56592; h_arr_right[counter] = 27'd68332; end
        8'd42 : begin  h_arr_left[counter] = 27'd63742; h_arr_right[counter] = 27'd52106; end
        8'd43 : begin  h_arr_left[counter] = -27'd4404; h_arr_right[counter] = -27'd481; end
        8'd44 : begin  h_arr_left[counter] = 27'd25587; h_arr_right[counter] = 27'd32484; end
        8'd45 : begin  h_arr_left[counter] = 27'd26537; h_arr_right[counter] = 27'd25703; end
        8'd46 : begin  h_arr_left[counter] = -27'd47297; h_arr_right[counter] = -27'd67441; end
        8'd47 : begin  h_arr_left[counter] = -27'd75019; h_arr_right[counter] = -27'd80883; end
        8'd48 : begin  h_arr_left[counter] = -27'd30435; h_arr_right[counter] = -27'd8315; end
        8'd49 : begin  h_arr_left[counter] = -27'd9375; h_arr_right[counter] = -27'd14422; end
        8'd50 : begin  h_arr_left[counter] = -27'd19611; h_arr_right[counter] = -27'd15349; end
        8'd51 : begin  h_arr_left[counter] = 27'd4234; h_arr_right[counter] = 27'd14056; end
        8'd52 : begin  h_arr_left[counter] = 27'd28747; h_arr_right[counter] = 27'd21222; end
        8'd53 : begin  h_arr_left[counter] = 27'd16538; h_arr_right[counter] = 27'd123; end
        8'd54 : begin  h_arr_left[counter] = -27'd9173; h_arr_right[counter] = 27'd5059; end
        8'd55 : begin  h_arr_left[counter] = 27'd13660; h_arr_right[counter] = 27'd13961; end
        8'd56 : begin  h_arr_left[counter] = -27'd1220; h_arr_right[counter] = -27'd22888; end
        8'd57 : begin  h_arr_left[counter] = -27'd37489; h_arr_right[counter] = -27'd32771; end
        8'd58 : begin  h_arr_left[counter] = -27'd8710; h_arr_right[counter] = 27'd4744; end
        8'd59 : begin  h_arr_left[counter] = 27'd8726; h_arr_right[counter] = 27'd14128; end
        8'd60 : begin  h_arr_left[counter] = -27'd9483; h_arr_right[counter] = -27'd11345; end
        8'd61 : begin  h_arr_left[counter] = -27'd6276; h_arr_right[counter] = -27'd5679; end
        8'd62 : begin  h_arr_left[counter] = 27'd5691; h_arr_right[counter] = 27'd11898; end
        8'd63 : begin  h_arr_left[counter] = 27'd8889; h_arr_right[counter] = 27'd10683; end
        8'd64 : begin  h_arr_left[counter] = 27'd3878; h_arr_right[counter] = -27'd3797; end
        8'd65 : begin  h_arr_left[counter] = -27'd940; h_arr_right[counter] = -27'd2341; end
        8'd66 : begin  h_arr_left[counter] = 27'd7523; h_arr_right[counter] = 27'd8526; end
        8'd67 : begin  h_arr_left[counter] = 27'd4317; h_arr_right[counter] = -27'd841; end
        8'd68 : begin  h_arr_left[counter] = -27'd12623; h_arr_right[counter] = -27'd9255; end
        8'd69 : begin  h_arr_left[counter] = -27'd9388; h_arr_right[counter] = -27'd5268; end
        8'd70 : begin  h_arr_left[counter] = -27'd865; h_arr_right[counter] = -27'd11193; end
        8'd71 : begin  h_arr_left[counter] = -27'd11980; h_arr_right[counter] = -27'd20115; end
        8'd72 : begin  h_arr_left[counter] = -27'd21087; h_arr_right[counter] = -27'd16952; end
        8'd73 : begin  h_arr_left[counter] = -27'd11515; h_arr_right[counter] = -27'd3635; end
        8'd74 : begin  h_arr_left[counter] = -27'd1024; h_arr_right[counter] = -27'd3045; end
        8'd75 : begin  h_arr_left[counter] = -27'd6321; h_arr_right[counter] = -27'd9744; end
        8'd76 : begin  h_arr_left[counter] = -27'd4796; h_arr_right[counter] = -27'd4215; end
        8'd77 : begin  h_arr_left[counter] = 27'd494; h_arr_right[counter] = 27'd4234; end
        8'd78 : begin  h_arr_left[counter] = -27'd1753; h_arr_right[counter] = 27'd3853; end
        8'd79 : begin  h_arr_left[counter] = -27'd3861; h_arr_right[counter] = -27'd1117; end
        8'd80 : begin  h_arr_left[counter] = -27'd3445; h_arr_right[counter] = -27'd3889; end
        8'd81 : begin  h_arr_left[counter] = 27'd1340; h_arr_right[counter] = -27'd2198; end
        8'd82 : begin  h_arr_left[counter] = 27'd1701; h_arr_right[counter] = 27'd951; end
        8'd83 : begin  h_arr_left[counter] = -27'd977; h_arr_right[counter] = -27'd588; end
        8'd84 : begin  h_arr_left[counter] = 27'd1109; h_arr_right[counter] = -27'd1760; end
        8'd85 : begin  h_arr_left[counter] = 27'd1281; h_arr_right[counter] = 27'd1127; end
        8'd86 : begin  h_arr_left[counter] = 27'd1405; h_arr_right[counter] = 27'd4015; end
        8'd87 : begin  h_arr_left[counter] = 27'd2624; h_arr_right[counter] = 27'd3947; end
        8'd88 : begin  h_arr_left[counter] = 27'd5071; h_arr_right[counter] = 27'd5336; end
        8'd89 : begin  h_arr_left[counter] = 27'd8583; h_arr_right[counter] = 27'd2057; end
        8'd90 : begin  h_arr_left[counter] = 27'd2407; h_arr_right[counter] = -27'd2492; end
        8'd91 : begin  h_arr_left[counter] = -27'd3876; h_arr_right[counter] = -27'd2151; end
        8'd92 : begin  h_arr_left[counter] = -27'd2042; h_arr_right[counter] = -27'd435; end
        8'd93 : begin  h_arr_left[counter] = -27'd4603; h_arr_right[counter] = -27'd2221; end
        8'd94 : begin  h_arr_left[counter] = -27'd11119; h_arr_right[counter] = -27'd4140; end
        8'd95 : begin  h_arr_left[counter] = -27'd9683; h_arr_right[counter] = -27'd2281; end
        8'd96 : begin  h_arr_left[counter] = -27'd6291; h_arr_right[counter] = -27'd1102; end
        8'd97 : begin  h_arr_left[counter] = -27'd5325; h_arr_right[counter] = -27'd2534; end
        8'd98 : begin  h_arr_left[counter] = -27'd218; h_arr_right[counter] = -27'd2605; end
        8'd99 : begin  h_arr_left[counter] = 27'd4542; h_arr_right[counter] = 27'd1718; end
        8'd100 : begin  h_arr_left[counter] = 27'd3709; h_arr_right[counter] = 27'd2200; end
        8'd101 : begin  h_arr_left[counter] = 27'd4470; h_arr_right[counter] = -27'd524; end
        8'd102 : begin  h_arr_left[counter] = 27'd5876; h_arr_right[counter] = -27'd322; end
        8'd103 : begin  h_arr_left[counter] = 27'd2082; h_arr_right[counter] = 27'd1043; end
        8'd104 : begin  h_arr_left[counter] = 27'd310; h_arr_right[counter] = 27'd752; end
        8'd105 : begin  h_arr_left[counter] = 27'd255; h_arr_right[counter] = 27'd2870; end
        8'd106 : begin  h_arr_left[counter] = -27'd1560; h_arr_right[counter] = 27'd3522; end
        8'd107 : begin  h_arr_left[counter] = 27'd95; h_arr_right[counter] = 27'd562; end
        8'd108 : begin  h_arr_left[counter] = 27'd2239; h_arr_right[counter] = 27'd733; end
        8'd109 : begin  h_arr_left[counter] = -27'd1462; h_arr_right[counter] = 27'd2755; end
        8'd110 : begin  h_arr_left[counter] = -27'd871; h_arr_right[counter] = 27'd188; end
        8'd111 : begin  h_arr_left[counter] = 27'd4073; h_arr_right[counter] = -27'd993; end
        8'd112 : begin  h_arr_left[counter] = 27'd2441; h_arr_right[counter] = 27'd715; end
        8'd113 : begin  h_arr_left[counter] = 27'd740; h_arr_right[counter] = 27'd508; end
        8'd114 : begin  h_arr_left[counter] = 27'd1418; h_arr_right[counter] = -27'd465; end
        8'd115 : begin  h_arr_left[counter] = -27'd913; h_arr_right[counter] = -27'd446; end
        8'd116 : begin  h_arr_left[counter] = -27'd1839; h_arr_right[counter] = -27'd1600; end
        8'd117 : begin  h_arr_left[counter] = 27'd844; h_arr_right[counter] = -27'd2029; end
        8'd118 : begin  h_arr_left[counter] = 27'd1172; h_arr_right[counter] = 27'd1111; end
        8'd119 : begin  h_arr_left[counter] = -27'd2720; h_arr_right[counter] = 27'd1627; end
        8'd120 : begin  h_arr_left[counter] = -27'd3348; h_arr_right[counter] = -27'd1295; end
        8'd121 : begin  h_arr_left[counter] = -27'd1784; h_arr_right[counter] = -27'd3236; end
        8'd122 : begin  h_arr_left[counter] = -27'd2366; h_arr_right[counter] = -27'd1576; end
        8'd123 : begin  h_arr_left[counter] = -27'd1196; h_arr_right[counter] = -27'd275; end
        8'd124 : begin  h_arr_left[counter] = 27'd666; h_arr_right[counter] = 27'd403; end
        8'd125 : begin  h_arr_left[counter] = -27'd529; h_arr_right[counter] = -27'd378; end
        8'd126 : begin  h_arr_left[counter] = -27'd754; h_arr_right[counter] = -27'd854; end
        8'd127 : begin  h_arr_left[counter] = 27'd2195; h_arr_right[counter] = 27'd1187; end
        8'd128 : begin  h_arr_left[counter] = 27'd1772; h_arr_right[counter] = 27'd3087; end
        8'd129 : begin  h_arr_left[counter] = 27'd1976; h_arr_right[counter] = 27'd2651; end
        8'd130 : begin  h_arr_left[counter] = 27'd3641; h_arr_right[counter] = 27'd1996; end
        8'd131 : begin  h_arr_left[counter] = 27'd1962; h_arr_right[counter] = 27'd2008; end
        8'd132 : begin  h_arr_left[counter] = 27'd743; h_arr_right[counter] = 27'd2057; end
        8'd133 : begin  h_arr_left[counter] = 27'd999; h_arr_right[counter] = 27'd2115; end
        8'd134 : begin  h_arr_left[counter] = 27'd124; h_arr_right[counter] = 27'd1570; end
        8'd135 : begin  h_arr_left[counter] = 27'd48; h_arr_right[counter] = 27'd583; end
        8'd136 : begin  h_arr_left[counter] = 27'd1474; h_arr_right[counter] = 27'd398; end
        8'd137 : begin  h_arr_left[counter] = 27'd721; h_arr_right[counter] = 27'd1174; end
        8'd138 : begin  h_arr_left[counter] = -27'd431; h_arr_right[counter] = 27'd1061; end
        8'd139 : begin  h_arr_left[counter] = 27'd261; h_arr_right[counter] = 27'd872; end
        8'd140 : begin  h_arr_left[counter] = 27'd1102; h_arr_right[counter] = 27'd164; end
        8'd141 : begin  h_arr_left[counter] = 27'd1288; h_arr_right[counter] = 27'd672; end
        8'd142 : begin  h_arr_left[counter] = 27'd814; h_arr_right[counter] = 27'd1822; end
        8'd143 : begin  h_arr_left[counter] = -27'd200; h_arr_right[counter] = 27'd421; end
        8'd144 : begin  h_arr_left[counter] = -27'd392; h_arr_right[counter] = -27'd619; end
        8'd145 : begin  h_arr_left[counter] = -27'd667; h_arr_right[counter] = -27'd431; end
        8'd146 : begin  h_arr_left[counter] = -27'd291; h_arr_right[counter] = -27'd383; end
        8'd147 : begin  h_arr_left[counter] = 27'd891; h_arr_right[counter] = 27'd841; end
        8'd148 : begin  h_arr_left[counter] = 27'd419; h_arr_right[counter] = 27'd1738; end
        8'd149 : begin  h_arr_left[counter] = 27'd1082; h_arr_right[counter] = 27'd1515; end
        8'd150 : begin  h_arr_left[counter] = 27'd1126; h_arr_right[counter] = 27'd1005; end
        8'd151 : begin  h_arr_left[counter] = -27'd592; h_arr_right[counter] = 27'd680; end
        8'd152 : begin  h_arr_left[counter] = 27'd494; h_arr_right[counter] = 27'd385; end
        8'd153 : begin  h_arr_left[counter] = 27'd890; h_arr_right[counter] = 27'd246; end
        8'd154 : begin  h_arr_left[counter] = 27'd53; h_arr_right[counter] = 27'd33; end
        8'd155 : begin  h_arr_left[counter] = 27'd1051; h_arr_right[counter] = -27'd71; end
        8'd156 : begin  h_arr_left[counter] = 27'd434; h_arr_right[counter] = 27'd987; end
        8'd157 : begin  h_arr_left[counter] = -27'd193; h_arr_right[counter] = 27'd1173; end
        8'd158 : begin  h_arr_left[counter] = 27'd778; h_arr_right[counter] = 27'd979; end
        8'd159 : begin  h_arr_left[counter] = 27'd920; h_arr_right[counter] = 27'd1153; end
        8'd160 : begin  h_arr_left[counter] = 27'd556; h_arr_right[counter] = 27'd571; end
        8'd161 : begin  h_arr_left[counter] = 27'd526; h_arr_right[counter] = 27'd392; end
        8'd162 : begin  h_arr_left[counter] = 27'd431; h_arr_right[counter] = 27'd91; end
        8'd163 : begin  h_arr_left[counter] = 27'd135; h_arr_right[counter] = -27'd460; end
        8'd164 : begin  h_arr_left[counter] = -27'd68; h_arr_right[counter] = -27'd95; end
        8'd165 : begin  h_arr_left[counter] = -27'd146; h_arr_right[counter] = 27'd108; end
        8'd166 : begin  h_arr_left[counter] = 27'd422; h_arr_right[counter] = 27'd509; end
        8'd167 : begin  h_arr_left[counter] = 27'd624; h_arr_right[counter] = 27'd1250; end
        8'd168 : begin  h_arr_left[counter] = 27'd298; h_arr_right[counter] = 27'd1206; end
        8'd169 : begin  h_arr_left[counter] = 27'd698; h_arr_right[counter] = 27'd1081; end
        8'd170 : begin  h_arr_left[counter] = 27'd873; h_arr_right[counter] = 27'd1224; end
        8'd171 : begin  h_arr_left[counter] = 27'd623; h_arr_right[counter] = 27'd1070; end
        8'd172 : begin  h_arr_left[counter] = 27'd702; h_arr_right[counter] = 27'd736; end
        8'd173 : begin  h_arr_left[counter] = 27'd275; h_arr_right[counter] = 27'd289; end
        8'd174 : begin  h_arr_left[counter] = -27'd30; h_arr_right[counter] = -27'd123; end
        8'd175 : begin  h_arr_left[counter] = 27'd196; h_arr_right[counter] = -27'd156; end
        8'd176 : begin  h_arr_left[counter] = 27'd103; h_arr_right[counter] = 27'd34; end
        8'd177 : begin  h_arr_left[counter] = 27'd384; h_arr_right[counter] = 27'd316; end
        8'd178 : begin  h_arr_left[counter] = 27'd559; h_arr_right[counter] = 27'd537; end
        8'd179 : begin  h_arr_left[counter] = 27'd430; h_arr_right[counter] = 27'd605; end
        8'd180 : begin  h_arr_left[counter] = 27'd519; h_arr_right[counter] = 27'd558; end
        8'd181 : begin  h_arr_left[counter] = 27'd330; h_arr_right[counter] = 27'd437; end
        8'd182 : begin  h_arr_left[counter] = 27'd117; h_arr_right[counter] = 27'd503; end
        8'd183 : begin  h_arr_left[counter] = 27'd170; h_arr_right[counter] = 27'd300; end
        8'd184 : begin  h_arr_left[counter] = 27'd135; h_arr_right[counter] = 27'd114; end
        8'd185 : begin  h_arr_left[counter] = 27'd95; h_arr_right[counter] = 27'd260; end
        8'd186 : begin  h_arr_left[counter] = 27'd209; h_arr_right[counter] = 27'd277; end
        8'd187 : begin  h_arr_left[counter] = 27'd175; h_arr_right[counter] = 27'd196; end
        8'd188 : begin  h_arr_left[counter] = 27'd167; h_arr_right[counter] = 27'd200; end
        8'd189 : begin  h_arr_left[counter] = 27'd217; h_arr_right[counter] = 27'd280; end
        8'd190 : begin  h_arr_left[counter] = 27'd142; h_arr_right[counter] = 27'd113; end
        8'd191 : begin  h_arr_left[counter] = 27'd116; h_arr_right[counter] = 27'd126; end
        8'd192 : begin  h_arr_left[counter] = 27'd80; h_arr_right[counter] = 27'd208; end
        8'd193 : begin  h_arr_left[counter] = 27'd16; h_arr_right[counter] = 27'd55; end
        8'd194 : begin  h_arr_left[counter] = -27'd152; h_arr_right[counter] = -27'd6; end
        8'd195 : begin  h_arr_left[counter] = -27'd288; h_arr_right[counter] = -27'd82; end
        8'd196 : begin  h_arr_left[counter] = -27'd127; h_arr_right[counter] = -27'd141; end
        8'd197 : begin  h_arr_left[counter] = 27'd12; h_arr_right[counter] = -27'd53; end
        8'd198 : begin  h_arr_left[counter] = -27'd44; h_arr_right[counter] = 27'd43; end
        8'd199 : begin  h_arr_left[counter] = 27'd119; h_arr_right[counter] = 27'd13; first = 0; end
    endcase
    end
        
    end
end
endmodule



module score(clk, reset, output_wire, output_wire2, output_wire3, violin2_counter, m10k_wire, counter_wire, fir_done);
	input clk;
	input reset;
	input fir_done;
	
 	output signed [26:0] output_wire; //total output
	output signed [26:0] output_wire2;
	output signed [26:0] output_wire3;
	output [19:0] violin2_counter;
	output [7:0] m10k_wire;
	output [10:0] counter_wire;

	wire [7:0] m10k_wire1;
	wire [10:0] counter_wire1;
	wire [7:0] m10k_wire2;
	wire [10:0] counter_wire2;
	
	wire [19:0] bass2_counter;
	wire [19:0] flute2_counter;

	wire signed [26:0] violin; //violin output
	wire signed [26:0] bass; //bass output
	wire signed [26:0] flute; //flute output
	assign output_wire = flute; //bass;// + violin;
	assign output_wire2 = violin;
	assign output_wire3 = bass;
	
	reg violin_reset_drive;
	reg [10:0] violin_str_len;
	reg [19:0] violin_duration;

	reg bass_reset_drive;
	reg [10:0] bass_str_len;
	reg [19:0] bass_duration;
	
	reg flute_reset_drive;
	reg [10:0] flute_str_len;
	reg [19:0] flute_duration;

	reg [29:0] counter; 
	
	reg violin_slur;
	reg [1:0] instr1 = 2'd0;
	reg [1:0] instr2 = 2'd1;
	reg [1:0] instr3 = 2'd3;
	
	reg instr_change;
	wire globalReset = reset || instr_change;

	instrument DUT (.clk(clk), .globalReset(globalReset), .reset(violin_reset_drive), .output_wire(violin), .str_len(violin_str_len), .type(1), .duration(violin_duration), .counter2_wire(violin2_counter), .instr(instr1), .m10k_wire(m10k_wire), .counter_wire(counter_wire), .fir_done(fir_done), .slur(violin_slur)); //flute
	instrument DUT1 (.clk(clk), .globalReset(globalReset), .reset(bass_reset_drive), .output_wire(bass), .str_len(bass_str_len), .type(0), .duration(bass_duration), .counter2_wire(bass2_counter), .instr(instr2), .m10k_wire(m10k_wire1), .counter_wire(counter_wire1), .fir_done(fir_done), .slur(1'b0)); //bass
	instrument DUT2 (.clk(clk), .globalReset(globalReset), .reset(flute_reset_drive), .output_wire(flute), .str_len(flute_str_len), .type(0), .duration(flute_duration), .counter2_wire(flute2_counter), .instr(instr3), .m10k_wire(m10k_wire2), .counter_wire(counter_wire2), .fir_done(fir_done), .slur(1'b0)); //bass

	always @ (posedge clk) begin

		if (reset) begin
			counter <= 0;
			instr1 <= 2'd0;
			instr2 <= 2'd1;
		end
		
		else if (counter == 30'd530) begin 
			violin_slur <= 1'b0;
			violin_reset_drive <= 1;
			violin_str_len <= 11'd92; //C5
			violin_duration <= 20'd61714; //1.5 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd531) begin
			violin_reset_drive <= 0;
		end
		
		if (counter == 30'd530) begin 
			bass_reset_drive <= 1;
			bass_str_len <= 11'd366; //C5
			bass_duration <= 20'd61714; //1.5 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd531) begin
			bass_reset_drive <= 0;
		end
		
		if (counter == 30'd530) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd10286; //really half a beat//0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd531) begin
			flute_reset_drive <= 0;
		end

		if (counter == 30'd21102) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec = 6857;
		end
		else if (counter == 30'd21103) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd27959) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd27960) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd34817) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd34818) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd41675) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd10286; //really half a beat//0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd41676) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd60000) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd60001) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd66858) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd66859) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd73716) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd73717) begin
			flute_reset_drive <= 0;
		end

		if (counter == 30'd61714) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd97; //B4
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
			violin_slur <= 1'b1;
		end
		else if (counter == 30'd61715) begin
			violin_reset_drive <= 0;
		end

		if (counter == 30'd72001) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd92; //C5
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd72002) begin
			violin_reset_drive <= 0;
		end

		if (counter == 30'd82287) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd82; //D5
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd82288) begin
			violin_reset_drive <= 0;
		end

		if (counter == 30'd82287) begin
			bass_reset_drive <= 1;
			bass_str_len <= 11'd244; //G3
			bass_duration <= 20'd10286; //not used
		end
		else if (counter == 30'd82288) begin
			bass_reset_drive <= 0;
		end
		
		
		if (counter == 30'd81000) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd10286; //really half a beat//0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd81001) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd92574) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd92; //C5
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd92575) begin
			violin_reset_drive <= 0;
		end

		if (counter == 30'd102860) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd97; //B4
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd102861) begin
			violin_reset_drive <= 0;
		end
		
		if (counter == 30'd100000) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd10286; //really half a beat//0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd100001) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd113147) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd109; //A4
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd113148) begin
			violin_reset_drive <= 0;
		end

		if (counter == 30'd123434) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd92; //C5
			violin_duration <= 20'd1000; //less to give a pause//0.5 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd123435) begin
			violin_reset_drive <= 0;
		end
		
		if (counter == 30'd123434) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd10286; //really half a beat//0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd123434) begin
			flute_reset_drive <= 0;
		end

		if (counter == 30'd123434) begin
			bass_reset_drive <= 1;
			bass_str_len <= 11'd366; //C3
			bass_duration <= 20'd10286; //not used
		end
		else if (counter == 30'd123435) begin
			bass_reset_drive <= 0;
		end

		if (counter == 30'd144006) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd92; //C5
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
			violin_slur <= 1'b0;
		end
		else if (counter == 30'd144007) begin
			violin_reset_drive <= 0;
		end

		if (counter == 30'd154293) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd109; //A4
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
			violin_slur <= 1'b1;
		end
		else if (counter == 30'd154294) begin
			violin_reset_drive <= 0;
		end
		
		if (counter == 30'd144000) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd144001) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd150858) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd150859) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd157715) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd157716) begin
			flute_reset_drive <= 0;
		end

		if (counter == 30'd164580) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd92; //C5
			violin_duration <= 20'd2000;//20'd61714; //1.5 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd164581) begin
			violin_reset_drive <= 0;
		end
		
		if (counter == 30'd164570) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd10286; //really half a beat//0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd164571) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd185152) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd185153) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd192009) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd192010) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd198866) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd198867) begin
			flute_reset_drive <= 0;
		end

		if (counter == 30'd205724) begin
			bass_reset_drive <= 1;
			bass_str_len <= 11'd244; //G3
			bass_duration <= 20'd10286; //not used
		end
		else if (counter == 30'd205725) begin
			bass_reset_drive <= 0;
		end
		
		if (counter == 30'd205720) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd10286; //really half a beat//0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd205721) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd226000) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd226001) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd232858) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd232859) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd239715) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd239716) begin
			flute_reset_drive <= 0;
		end

		if (counter == 30'd226295) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd97; //B4
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
			violin_slur <= 1'b0;
		end
		else if (counter == 30'd226296) begin
			violin_reset_drive <= 0;
		end

		if (counter == 30'd236582) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd92; //C5
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
			violin_slur <= 1'b1;
		end
		else if (counter == 30'd236583) begin
			violin_reset_drive <= 0;
		end

		if (counter == 30'd246869) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd109; //A4
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd246870) begin
			violin_reset_drive <= 0;
		end
		
		if (counter == 30'd246750) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd10286; //really half a beat//0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd246751) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd267400) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd267401) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd274258) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd274259) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd281115) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd281116) begin
			flute_reset_drive <= 0;
		end
	
		if (counter == 30'd246869) begin
			bass_reset_drive <= 1;
			bass_str_len <= 11'd366; //C3
			bass_duration <= 20'd10286; //not used
		end
		else if (counter == 30'd246870) begin
			bass_reset_drive <= 0;
		end

		if (counter == 30'd257156) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd122; //G4
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd257157) begin
			violin_reset_drive <= 0;
		end

		if (counter == 30'd267443) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd145; //E4
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd267444) begin
			violin_reset_drive <= 0;
		end

		if (counter == 30'd277730) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd137; //F4
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd277731) begin
			violin_reset_drive <= 0;
		end

		if (counter == 30'd288017) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd122; //G4
			violin_duration <= 20'd15000;//20'd92571; //2.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd288018) begin
			violin_reset_drive <= 0;
		end
		
		if (counter == 30'd288000) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd10286; //really half a beat//0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd288001) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd308572) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd308573) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd315429) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd315430) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd322286) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd322287) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd329160) begin
			bass_reset_drive <= 1;
			bass_str_len <= 11'd244; //G3
			bass_duration <= 20'd10286; //not used
		end
		else if (counter == 30'd329161) begin
			bass_reset_drive <= 0;
		end
		
		if (counter == 30'd329100) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd10286; //really half a beat//0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd329101) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd349671) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd10286; //really half a beat//0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd349672) begin
			flute_reset_drive <= 0;
		end

		if (counter == 30'd370302) begin
			bass_reset_drive <= 1;
			bass_str_len <= 11'd367; //C3
			bass_duration <= 20'd10286; //not used
		end
		else if (counter == 30'd370303) begin
			bass_reset_drive <= 0;
		end
		
		if (counter == 30'd370200) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd10286; //really half a beat//0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd370201) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd390771) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd390772) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd397628) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd397629) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd404486) begin //62247
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd5000; //1/6 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd404487) begin
			flute_reset_drive <= 0;
		end

		if (counter == 30'd380589) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd137; //F4
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
			violin_slur <= 1'b0;
		end
		else if (counter == 30'd380590) begin
			violin_reset_drive <= 0;
		end
		
		if (counter == 30'd390876) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd145; //E4
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd390877) begin
			violin_reset_drive <= 0;
		end
		
		if (counter == 30'd401163) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd163; //D4
			violin_duration <= 20'd10286; //0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd401164) begin
			violin_reset_drive <= 0;
		end
		
		if (counter == 30'd411450) begin
			violin_reset_drive <= 1;
			violin_str_len <= 11'd183; //C4
			violin_duration <= 20'd20572; //0.5 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
			violin_slur <= 1'd0;
		end
		else if (counter == 30'd411451) begin
			violin_reset_drive <= 0;
		end
		
		if (counter == 30'd411000) begin 
			flute_reset_drive <= 1;
			flute_str_len <= 11'd122; //G4
			flute_duration <= 20'd10286; //really half a beat//0.25 beats * 1/70 min/beat * 60 sec/min * 48000 samples/sec;
		end
		else if (counter == 30'd411001) begin
			flute_reset_drive <= 0;
		end
		
		if (counter == 30'd452592) begin
			bass_reset_drive <= 1;
			bass_str_len <= 11'd244; //G3
			bass_duration <= 20'd10286; //not used
		end
		else if (counter == 30'd452593) begin
			bass_reset_drive <= 0;
		end

		if (counter == 30'd493735) begin
			bass_reset_drive <= 1;
			bass_str_len <= 11'd367; //C3
			bass_duration <= 20'd10286; //not used
		end
		else if (counter == 30'd493736) begin
			bass_reset_drive <= 0;
		end
		
		
		if ((!reset) && fir_done) begin
			counter <= counter + 1;
			
			if (counter==30'd2)
				instr_change <= 1'b0;
				
			if (counter==30'd543736) begin
				counter <= 30'd1; //loop back
				if (instr1==2'd0) begin
					instr1 <= 2'd1;
					//instr2 <= 2'd2;
				end else if (instr1==2'd1) begin
					instr1 <= 2'd3;
					//instr2 <= 2'd3;
				//end else if (instr1==2'd2) begin
					//instr1 <= 2'd3;
					//instr2 <= 2'd0;
				end else if (instr1==2'd3) begin
					instr1 <= 2'd0;
					//instr2 <= 2'd1;
				end
				else
					instr1 <= 2'd0;
				instr_change <= 1'b1; //change instruments
			end
		end

	end //end always
endmodule

module instrument(clk, globalReset, reset, output_wire, str_len, type, duration, counter2_wire, instr, m10k_wire, counter_wire, fir_done, slur);
	input clk;
	input globalReset; //calculates the noise
	input reset; //sets delay_line to noise
	input [10:0] str_len;
	input type; //type is pluck (0) or bow (1)
	input [19:0] duration;
	input [1:0] instr; //instrument: 0 if flute, 1 if string
	input fir_done;
	input slur;
	
	output signed [26:0] output_wire;
	output [7:0] m10k_wire;
	output [10:0] counter_wire;
	
	

	localparam resetState = 3'b000;
	localparam nextState = 3'b001;
	localparam globalResetState = 3'd2;
	localparam readState = 3'd3;
	localparam blankState = 3'd4;
	localparam blankState2 = 3'd5;
	localparam blankState3 = 3'd6;


	reg [2:0] state = resetState;
	reg [10:0] counter = 0; 
	
	reg [19:0] counter2 = 0; //amp modulation
	output [19:0] counter2_wire; //here for amplitude debugging
	assign counter2_wire = counter2;
	
	reg noise_round=1'd1; //1 if first time round a loop
	
	wire signed [19:0] noise_out_m10k_20;
	reg signed [19:0] noise_in_m10k;
	reg [8:0] wraddress = 9'd0;
	reg [8:0] rdaddress = 9'd0;
	reg wren, rden;
	reg [8:0] counter_m10k = 9'd0;
	reg first_time = 1'd1;

	assign m10k_wire = counter_m10k; //used for debugging
	assign counter_wire = counter; //used for debugging

	wire signed [19:0] delay_line_prev;
	reg [8:0] rdaddress1 = 9'd0;
	reg [8:0] wraddress1 = 9'd0;

	RAM_512_20 m10(.q(noise_out_m10k_20), .data(noise_in_m10k), .wraddress(wraddress), .rdaddress(rdaddress), .wren(wren), .rden(rden), .clock(clk));
	RAM_512_20 m10prev(.q(delay_line_prev), .data(out_reg), .wraddress(wraddress1), .rdaddress(rdaddress1), .wren(wren), .rden(rden), .clock(clk));

	reg signed [19:0] out_reg;
	wire signed [19:0] output_wire1;
	assign output_wire1 = (type && slur) ? (out_reg<<1) //bowed and noise round
								: type ? out_reg //just bowed
								: (out_reg<<<2); //plucked
	assign output_wire = output_wire[19] ? {7'b1111111, output_wire1} : {7'b0000000, output_wire1};
	
	always @(posedge clk) begin
	    if (globalReset) begin
				counter <= 11'd0;
				counter2 <= 20'd0;
				counter_m10k <= 9'd0;
				state <= globalResetState;
				wren <= 1'd1;
				rden <= 1'd0;
	    end
	    
	    else if (reset) begin
			counter <= str_len;
			counter_m10k <= str_len[8:0];
			counter2 <= 20'd1;
			noise_round <= 1'b1; //delay_line will be noise
			state<= resetState;
			wren <= 1'b0;
	    end
	    
	    //calculate the noise
	    else if (state==globalResetState) begin
				wraddress <= counter_m10k;
			if (instr==2'd0) begin //flute
				if (counter_m10k==9'd0) begin
					noise_in_m10k <= -20'd50;
				end
	         else if (counter_m10k<9'd50) begin
	            noise_in_m10k <= noise_in_m10k + 20'd10;
				end
            else begin
               noise_in_m10k <= noise_in_m10k - 20'd10;
				end
         end
        	else if (instr == 2'd1) begin //string
				if (counter_m10k==9'd0) begin
					noise_in_m10k <= 20'd10;
				end
				else begin
           		noise_in_m10k <= noise_in_m10k + 20'd10;
				end
				/*if (counter_m10k%2==0)
					noise_in_m10k <= 20'd1000;
				else
					noise_in_m10k <= -20'd1000;*/
				
         end
			else if (instr == 2'd2) begin
				if (counter_m10k==9'd0) begin
					noise_in_m10k <= 20'd1000;
				end
				else if (counter_m10k<9'd50) begin
	            noise_in_m10k <= noise_in_m10k - 20'd30;
				end
				else begin
           		noise_in_m10k <= noise_in_m10k + 20'd10;
				end
			end
			else if (instr == 2'd3) begin //string
				if (counter_m10k==9'd0) begin
					noise_in_m10k <= 20'd2000;
				end
				else if (counter_m10k<9'd10) begin
           		noise_in_m10k <= noise_in_m10k - 20'd50;
				end
				else if (counter_m10k==9'd20) begin
           		noise_in_m10k <= 20'd1500;
				end
				else if (counter_m10k<9'd30) begin
           		noise_in_m10k <= noise_in_m10k - 20'd10;
				end
				else if (counter_m10k==9'd40) begin
           		noise_in_m10k <= 20'd3000;
				end
				else if (counter_m10k<9'd50) begin
           		noise_in_m10k <= noise_in_m10k - 20'd70;
				end
				else if (counter_m10k==9'd60) begin
           		noise_in_m10k <= 20'd2000;
				end
				else if (counter_m10k<9'd70) begin
           		noise_in_m10k <= noise_in_m10k - 20'd70;
				end
				else if (counter_m10k==9'd80) begin
           		noise_in_m10k <= 20'd4000;
				end
				else if (counter_m10k<9'd90) begin
           		noise_in_m10k <= noise_in_m10k - 20'd10;
				end
				else if (counter_m10k==9'd100) begin
           		noise_in_m10k <= 20'd1000;
				end
				else if (counter_m10k<9'd110) begin
           		noise_in_m10k <= noise_in_m10k - 20'd20;
				end
				else if (counter_m10k==9'd120) begin
           		noise_in_m10k <= 20'd1000;
				end
				else if (counter_m10k<9'd130) begin
           		noise_in_m10k <= noise_in_m10k - 20'd0;
				end
				else if (counter_m10k==9'd140) begin
           		noise_in_m10k <= 20'd500;
				end
				else if (counter_m10k<9'd150) begin
           		noise_in_m10k <= noise_in_m10k - 10'd0;
				end
				else if (counter_m10k==9'd150) begin
           		noise_in_m10k <= 20'd2000;
				end
				else begin
           		noise_in_m10k <= noise_in_m10k - 10'd0;
				end 
				//noise_in_m10k <= noise_in_m10k>>>1;
         end

			if (counter_m10k >= 9'd511) begin //originally 377
             	//counter <= str_len; //DO NOT UNCOMMENT THIS
					//counter_m10k <= str_len[7:0]; //DO NOT UNCOMMENT THIS
				wren <= 1'b0;
	    	end
         else 
				counter_m10k <= counter_m10k + 9'd1;
                
	    end //end globalResetState
	    
	    //new note
		else if (state==resetState) begin
			counter2 <= 20'd1;
			noise_round <= 1'b1; //delay_line will be noise
			counter_m10k <= str_len[8:0];
			counter <= str_len;
			state<= readState;
		end

		else if (state==readState) begin
			wren <= 1'b0;
			rden <= 1'b1;
			rdaddress <= counter_m10k;
			wraddress1 <= counter_m10k;
			rdaddress1 <= counter_m10k;
			state <= blankState;
		end
		
		else if (state==blankState) begin
			state<=blankState2;
		end
		else if (state==blankState2) begin
			state<=blankState3;
		end
		else if (state==blankState3 && (first_time || fir_done)) begin
			//state<=nextState;
			if (noise_round) begin //noise_round is first round
			    rden <= 1'b1;
				if (type && slur)
					out_reg <= noise_out_m10k_20;//<<<4;
				else
					out_reg <= noise_out_m10k_20;
			end
			else if (type) begin//bow
					//ramp builds
					if (counter2<(duration>>>11)) begin	
						out_reg <= (out_reg>>>1) + (delay_line_prev>>>1) - (out_reg>>>8) - (delay_line_prev>>>6) + (noise_out_m10k_20>>>7);
					end else if (counter2<(duration>>>10)) begin
						out_reg <= (out_reg>>>1) + (delay_line_prev>>>1) - (out_reg>>>8) - (delay_line_prev>>>6) + (noise_out_m10k_20>>>6);
					end else if (counter2<(duration>>>9)) begin
						out_reg <= (out_reg>>>1) + (delay_line_prev>>>1) - (out_reg>>>8) - (delay_line_prev>>>6) + (noise_out_m10k_20>>>5);
					end else if (counter2<(duration>>>8)) begin	
						out_reg <= (out_reg>>>1) + (delay_line_prev>>>1) - (out_reg>>>8) - (delay_line_prev>>>6) + (noise_out_m10k_20>>>4);
					end else if (counter2<(duration>>>7)) begin			
						out_reg <= (out_reg>>>1) + (delay_line_prev>>>1) - (out_reg>>>8) - (delay_line_prev>>>6) + (noise_out_m10k_20>>>3);
					end else if (counter2<(duration>>>6)) begin
						out_reg <= (out_reg>>>1) + (delay_line_prev>>>1) - (out_reg>>>8) - (delay_line_prev>>>6) + (noise_out_m10k_20>>>2);
					end else if (counter2<(duration>>>4)) begin
						out_reg <= (out_reg>>>1) + (delay_line_prev>>>1) - (out_reg>>>8) - (delay_line_prev>>>6) + (noise_out_m10k_20>>>1);
					//ramp constant
					end else if (counter2<duration) begin	
						out_reg <= (out_reg>>>1) + (delay_line_prev>>>1) - (out_reg>>>8) - (delay_line_prev>>>6) + (noise_out_m10k_20);	
					//ramp decays, no longer feeding noise
					end else begin 
						out_reg <= (out_reg>>>1) + (delay_line_prev>>>1) - (out_reg>>>8) - (delay_line_prev>>>6);
					end
				
			end //end type bow

			else if (!type) begin
				if (counter==str_len) begin
					out_reg <= (out_reg>>>1) + (delay_line_prev>>>1) - (out_reg>>>11) - (delay_line_prev>>>11);
				end else begin
					out_reg <= (out_reg>>>1) + (delay_line_prev>>>1) - (out_reg>>>11) - (delay_line_prev>>>11);
				end
			end //end type pizz
			rden <= 1'b0;
			wren <= 1'b1;

			if (first_time)
				first_time <= 0;

			if (counter==0) begin
			    	if (noise_round) 
			    	    noise_round <= 0;
				counter <= str_len;
				counter_m10k <= str_len[8:0];
			end
			else begin
				counter <= counter-10'd1;
				counter_m10k <= counter_m10k-9'd1;
			end

			if (counter2!=0) begin //didn't overflow
				counter2<= counter2 + 20'd1;
			end

			state <= readState;
		end
		
	end //end always
	
endmodule

module RAM_512_20(
	output reg signed [19:0] q,
	input signed [19:0] data,
	input [8:0] wraddress, rdaddress,
	input wren, rden, clock
);

	reg [8:0] read_address_reg;
	reg signed [19:0] mem [511:0];
	reg rden_reg;
	
	always @ (posedge clock)
	begin
		if (wren) 
			mem[wraddress] <= data;
	end
	always @ (posedge clock) begin
		if (rden_reg)
			q<= mem[read_address_reg]; //don't write to any reg's to avoid a second blank state
		read_address_reg <= rdaddress;
		rden_reg <= rden;
	end
	
endmodule




//////////////////////////////////////////////////
////////////	Sin Wave ROM Table	//////////////
//////////////////////////////////////////////////
// produces a 2's comp, 16-bit, approximation
// of a sine wave, given an input phase (address)
module sync_rom (clock, address, sine, done);
input clock;
input [7:0] address;
input done;
output [15:0] sine;
reg signed [15:0] sine;
always@(posedge clock)
begin

    case(address)
    		8'h00: sine = 16'h0000 ;
			8'h01: sine = 16'h0192 ;
			8'h02: sine = 16'h0323 ;
			8'h03: sine = 16'h04b5 ;
			8'h04: sine = 16'h0645 ;
			8'h05: sine = 16'h07d5 ;
			8'h06: sine = 16'h0963 ;
			8'h07: sine = 16'h0af0 ;
			8'h08: sine = 16'h0c7c ;
			8'h09: sine = 16'h0e05 ;
			8'h0a: sine = 16'h0f8c ;
			8'h0b: sine = 16'h1111 ;
			8'h0c: sine = 16'h1293 ;
			8'h0d: sine = 16'h1413 ;
			8'h0e: sine = 16'h158f ;
			8'h0f: sine = 16'h1708 ;
			8'h10: sine = 16'h187d ;
			8'h11: sine = 16'h19ef ;
			8'h12: sine = 16'h1b5c ;
			8'h13: sine = 16'h1cc5 ;
			8'h14: sine = 16'h1e2a ;
			8'h15: sine = 16'h1f8b ;
			8'h16: sine = 16'h20e6 ;
			8'h17: sine = 16'h223c ;
			8'h18: sine = 16'h238d ;
			8'h19: sine = 16'h24d9 ;
			8'h1a: sine = 16'h261f ;
			8'h1b: sine = 16'h275f ;
			8'h1c: sine = 16'h2899 ;
			8'h1d: sine = 16'h29cc ;
			8'h1e: sine = 16'h2afa ;
			8'h1f: sine = 16'h2c20 ;
			8'h20: sine = 16'h2d40 ;
			8'h21: sine = 16'h2e59 ;
			8'h22: sine = 16'h2f6b ;
			8'h23: sine = 16'h3075 ;
			8'h24: sine = 16'h3178 ;
			8'h25: sine = 16'h3273 ;
			8'h26: sine = 16'h3366 ;
			8'h27: sine = 16'h3452 ;
			8'h28: sine = 16'h3535 ;
			8'h29: sine = 16'h3611 ;
			8'h2a: sine = 16'h36e4 ;
			8'h2b: sine = 16'h37ae ;
			8'h2c: sine = 16'h3870 ;
			8'h2d: sine = 16'h3929 ;
			8'h2e: sine = 16'h39da ;
			8'h2f: sine = 16'h3a81 ;
			8'h30: sine = 16'h3b1f ;
			8'h31: sine = 16'h3bb5 ;
			8'h32: sine = 16'h3c41 ;
			8'h33: sine = 16'h3cc4 ;
			8'h34: sine = 16'h3d3d ;
			8'h35: sine = 16'h3dad ;
			8'h36: sine = 16'h3e14 ;
			8'h37: sine = 16'h3e70 ;
			8'h38: sine = 16'h3ec4 ;
			8'h39: sine = 16'h3f0d ;
			8'h3a: sine = 16'h3f4d ;
			8'h3b: sine = 16'h3f83 ;
			8'h3c: sine = 16'h3fb0 ;
			8'h3d: sine = 16'h3fd2 ;
			8'h3e: sine = 16'h3feb ;
			8'h3f: sine = 16'h3ffa ;
			8'h40: sine = 16'h3fff ;
			8'h41: sine = 16'h3ffa ;
			8'h42: sine = 16'h3feb ;
			8'h43: sine = 16'h3fd2 ;
			8'h44: sine = 16'h3fb0 ;
			8'h45: sine = 16'h3f83 ;
			8'h46: sine = 16'h3f4d ;
			8'h47: sine = 16'h3f0d ;
			8'h48: sine = 16'h3ec4 ;
			8'h49: sine = 16'h3e70 ;
			8'h4a: sine = 16'h3e14 ;
			8'h4b: sine = 16'h3dad ;
			8'h4c: sine = 16'h3d3d ;
			8'h4d: sine = 16'h3cc4 ;
			8'h4e: sine = 16'h3c41 ;
			8'h4f: sine = 16'h3bb5 ;
			8'h50: sine = 16'h3b1f ;
			8'h51: sine = 16'h3a81 ;
			8'h52: sine = 16'h39da ;
			8'h53: sine = 16'h3929 ;
			8'h54: sine = 16'h3870 ;
			8'h55: sine = 16'h37ae ;
			8'h56: sine = 16'h36e4 ;
			8'h57: sine = 16'h3611 ;
			8'h58: sine = 16'h3535 ;
			8'h59: sine = 16'h3452 ;
			8'h5a: sine = 16'h3366 ;
			8'h5b: sine = 16'h3273 ;
			8'h5c: sine = 16'h3178 ;
			8'h5d: sine = 16'h3075 ;
			8'h5e: sine = 16'h2f6b ;
			8'h5f: sine = 16'h2e59 ;
			8'h60: sine = 16'h2d40 ;
			8'h61: sine = 16'h2c20 ;
			8'h62: sine = 16'h2afa ;
			8'h63: sine = 16'h29cc ;
			8'h64: sine = 16'h2899 ;
			8'h65: sine = 16'h275f ;
			8'h66: sine = 16'h261f ;
			8'h67: sine = 16'h24d9 ;
			8'h68: sine = 16'h238d ;
			8'h69: sine = 16'h223c ;
			8'h6a: sine = 16'h20e6 ;
			8'h6b: sine = 16'h1f8b ;
			8'h6c: sine = 16'h1e2a ;
			8'h6d: sine = 16'h1cc5 ;
			8'h6e: sine = 16'h1b5c ;
			8'h6f: sine = 16'h19ef ;
			8'h70: sine = 16'h187d ;
			8'h71: sine = 16'h1708 ;
			8'h72: sine = 16'h158f ;
			8'h73: sine = 16'h1413 ;
			8'h74: sine = 16'h1293 ;
			8'h75: sine = 16'h1111 ;
			8'h76: sine = 16'h0f8c ;
			8'h77: sine = 16'h0e05 ;
			8'h78: sine = 16'h0c7c ;
			8'h79: sine = 16'h0af0 ;
			8'h7a: sine = 16'h0963 ;
			8'h7b: sine = 16'h07d5 ;
			8'h7c: sine = 16'h0645 ;
			8'h7d: sine = 16'h04b5 ;
			8'h7e: sine = 16'h0323 ;
			8'h7f: sine = 16'h0192 ;
			8'h80: sine = 16'h0000 ;
			8'h81: sine = 16'hfe6e ;
			8'h82: sine = 16'hfcdd ;
			8'h83: sine = 16'hfb4b ;
			8'h84: sine = 16'hf9bb ;
			8'h85: sine = 16'hf82b ;
			8'h86: sine = 16'hf69d ;
			8'h87: sine = 16'hf510 ;
			8'h88: sine = 16'hf384 ;
			8'h89: sine = 16'hf1fb ;
			8'h8a: sine = 16'hf074 ;
			8'h8b: sine = 16'heeef ;
			8'h8c: sine = 16'hed6d ;
			8'h8d: sine = 16'hebed ;
			8'h8e: sine = 16'hea71 ;
			8'h8f: sine = 16'he8f8 ;
			8'h90: sine = 16'he783 ;
			8'h91: sine = 16'he611 ;
			8'h92: sine = 16'he4a4 ;
			8'h93: sine = 16'he33b ;
			8'h94: sine = 16'he1d6 ;
			8'h95: sine = 16'he075 ;
			8'h96: sine = 16'hdf1a ;
			8'h97: sine = 16'hddc4 ;
			8'h98: sine = 16'hdc73 ;
			8'h99: sine = 16'hdb27 ;
			8'h9a: sine = 16'hd9e1 ;
			8'h9b: sine = 16'hd8a1 ;
			8'h9c: sine = 16'hd767 ;
			8'h9d: sine = 16'hd634 ;
			8'h9e: sine = 16'hd506 ;
			8'h9f: sine = 16'hd3e0 ;
			8'ha0: sine = 16'hd2c0 ;
			8'ha1: sine = 16'hd1a7 ;
			8'ha2: sine = 16'hd095 ;
			8'ha3: sine = 16'hcf8b ;
			8'ha4: sine = 16'hce88 ;
			8'ha5: sine = 16'hcd8d ;
			8'ha6: sine = 16'hcc9a ;
			8'ha7: sine = 16'hcbae ;
			8'ha8: sine = 16'hcacb ;
			8'ha9: sine = 16'hc9ef ;
			8'haa: sine = 16'hc91c ;
			8'hab: sine = 16'hc852 ;
			8'hac: sine = 16'hc790 ;
			8'had: sine = 16'hc6d7 ;
			8'hae: sine = 16'hc626 ;
			8'haf: sine = 16'hc57f ;
			8'hb0: sine = 16'hc4e1 ;
			8'hb1: sine = 16'hc44b ;
			8'hb2: sine = 16'hc3bf ;
			8'hb3: sine = 16'hc33c ;
			8'hb4: sine = 16'hc2c3 ;
			8'hb5: sine = 16'hc253 ;
			8'hb6: sine = 16'hc1ec ;
			8'hb7: sine = 16'hc190 ;
			8'hb8: sine = 16'hc13c ;
			8'hb9: sine = 16'hc0f3 ;
			8'hba: sine = 16'hc0b3 ;
			8'hbb: sine = 16'hc07d ;
			8'hbc: sine = 16'hc050 ;
			8'hbd: sine = 16'hc02e ;
			8'hbe: sine = 16'hc015 ;
			8'hbf: sine = 16'hc006 ;
			8'hc0: sine = 16'hc001 ;
			8'hc1: sine = 16'hc006 ;
			8'hc2: sine = 16'hc015 ;
			8'hc3: sine = 16'hc02e ;
			8'hc4: sine = 16'hc050 ;
			8'hc5: sine = 16'hc07d ;
			8'hc6: sine = 16'hc0b3 ;
			8'hc7: sine = 16'hc0f3 ;
			8'hc8: sine = 16'hc13c ;
			8'hc9: sine = 16'hc190 ;
			8'hca: sine = 16'hc1ec ;
			8'hcb: sine = 16'hc253 ;
			8'hcc: sine = 16'hc2c3 ;
			8'hcd: sine = 16'hc33c ;
			8'hce: sine = 16'hc3bf ;
			8'hcf: sine = 16'hc44b ;
			8'hd0: sine = 16'hc4e1 ;
			8'hd1: sine = 16'hc57f ;
			8'hd2: sine = 16'hc626 ;
			8'hd3: sine = 16'hc6d7 ;
			8'hd4: sine = 16'hc790 ;
			8'hd5: sine = 16'hc852 ;
			8'hd6: sine = 16'hc91c ;
			8'hd7: sine = 16'hc9ef ;
			8'hd8: sine = 16'hcacb ;
			8'hd9: sine = 16'hcbae ;
			8'hda: sine = 16'hcc9a ;
			8'hdb: sine = 16'hcd8d ;
			8'hdc: sine = 16'hce88 ;
			8'hdd: sine = 16'hcf8b ;
			8'hde: sine = 16'hd095 ;
			8'hdf: sine = 16'hd1a7 ;
			8'he0: sine = 16'hd2c0 ;
			8'he1: sine = 16'hd3e0 ;
			8'he2: sine = 16'hd506 ;
			8'he3: sine = 16'hd634 ;
			8'he4: sine = 16'hd767 ;
			8'he5: sine = 16'hd8a1 ;
			8'he6: sine = 16'hd9e1 ;
			8'he7: sine = 16'hdb27 ;
			8'he8: sine = 16'hdc73 ;
			8'he9: sine = 16'hddc4 ;
			8'hea: sine = 16'hdf1a ;
			8'heb: sine = 16'he075 ;
			8'hec: sine = 16'he1d6 ;
			8'hed: sine = 16'he33b ;
			8'hee: sine = 16'he4a4 ;
			8'hef: sine = 16'he611 ;
			8'hf0: sine = 16'he783 ;
			8'hf1: sine = 16'he8f8 ;
			8'hf2: sine = 16'hea71 ;
			8'hf3: sine = 16'hebed ;
			8'hf4: sine = 16'hed6d ;
			8'hf5: sine = 16'heeef ;
			8'hf6: sine = 16'hf074 ;
			8'hf7: sine = 16'hf1fb ;
			8'hf8: sine = 16'hf384 ;
			8'hf9: sine = 16'hf510 ;
			8'hfa: sine = 16'hf69d ;
			8'hfb: sine = 16'hf82b ;
			8'hfc: sine = 16'hf9bb ;
			8'hfd: sine = 16'hfb4b ;
			8'hfe: sine = 16'hfcdd ;
			8'hff: sine = 16'hfe6e ;
	endcase
	end
	

endmodule
