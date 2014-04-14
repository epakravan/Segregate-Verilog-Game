`timescale 1ns / 1ps

module segregate_top(MemOE, MemWR, RamCS, FlashCS, QuadSpiFlashCS,
      ClkPort,                           // the 100 MHz incoming clock signal
		BtnL, BtnU, BtnR,            // the Left, Up, Down, and the Right buttons BtnL, BtnR,
		BtnC,                              // the center button (this is our reset in most of our designs)
		Ld7, Ld6, Ld5, Ld4, Ld3, Ld2, Ld1, Ld0, // 8 LEDs
		An3, An2, An1, An0,			       // 4 anodes
		Ca, Cb, Cc, Cd, Ce, Cf, Cg,        // 7 cathodes
		Dp,                                 // Dot Point Cathode on SSDs
		vga_h_sync, vga_v_sync,			//VGA Sync
		vga_r, vga_g, vga_b 				//VGA Colors
		);


	/*  INPUTS */
	// Clock & Reset I/O
	input		ClkPort;	
	// Project Specific Inputs
	input		BtnL, BtnU, BtnR, BtnC;	
	
	parameter N = 28'd268435455;
	
	wire Pulse;
	
	/*  OUTPUTS */
	// Control signals on Memory chips
	output 	MemOE, MemWR, RamCS, FlashCS, QuadSpiFlashCS;
	// Project Specific Outputs
	// LEDs
	output 	Ld0, Ld1, Ld2, Ld3, Ld4, Ld5, Ld6, Ld7;
	// SSD Outputs
	output 	Cg, Cf, Ce, Cd, Cc, Cb, Ca, Dp;
	output 	An0, An1, An2, An3;	
	output vga_h_sync, vga_v_sync, vga_r, vga_g, vga_b;
	reg vga_r, vga_g, vga_b;
	
	/*  LOCAL SIGNALS */
	wire		Reset, ClkPort;
	wire		board_clk, sys_clk;
	wire [1:0] 	ssdscan_clk;
	reg [26:0]	DIV_CLK;
	wire Start_Ack_Pulse;
	wire LEFT_Pulse, RIGHT_Pulse, BtnR_Pulse, BtnL_Pulse;
	wire q_I, q_Play, q_Left, q_Right, q_Skip, q_Done;
	wire [6:0] score;
	wire [5:0] posR, posL;
	wire [63:0] lightR, lightL, colorR, colorL;
	wire temp;
	reg [3:0]	SSD;
	wire [3:0]	SSD3, SSD2, SSD1, SSD0; 
	reg [7:0]  SSD_CATHODES; 
	

	assign {MemOE, MemWR, RamCS, FlashCS, QuadSpiFlashCS} = 5'b11111;

 BUFGP BUFGP1 (board_clk, ClkPort); 

	assign Reset = BtnU;
	
  always @(posedge board_clk, posedge Reset) 	
    begin							
        if (Reset)
		DIV_CLK <= 0;
        else
		DIV_CLK <= DIV_CLK + 1'b1;
    end

	assign	sys_clk = board_clk;


ee201_debouncer #(.N_dc(28)) ee201_debouncer_1 
        (.CLK(sys_clk), .RESET(Reset), .PB(BtnC), .DPB( ), 
		.SCEN(Start_Ack_Pulse), .MCEN( ), .CCEN( ));
		 		 
	assign {LEFT_Pulse, RIGHT_Pulse} = {BtnL_Pulse, BtnR_Pulse};

ee201_debouncer #(.N_dc(28)) ee201_debouncer_2 
        (.CLK(sys_clk), .RESET(Reset), .PB(BtnR), .DPB( ), 
		.SCEN(BtnR_Pulse), .MCEN( ), .CCEN( ));
		
ee201_debouncer #(.N_dc(28)) ee201_debouncer_3 
        (.CLK(sys_clk), .RESET(Reset), .PB(BtnL), .DPB( ), 
		.SCEN(BtnL_Pulse), .MCEN( ), .CCEN( ));
		
	clock PGEN1 (
		.Clk(sys_clk), .Reset(Reset), .Pulse(Pulse), .N(N)
		
	);
		

	
	// the state machine module
	State thestates(.Clk(sys_clk), .rand(DIV_CLK[0]), .LEFT_Btn(LEFT_Pulse), .RIGHT_Btn(RIGHT_Pulse), .Reset(Reset), .Start(Start_Ack_Pulse), .Ack(Start_Ack_Pulse), .Pulse(Pulse),
						  .score(score), .posL(posL), .posR(posR), .lightL(lightL), .lightR(lightR), .colorL(colorL), .colorR(colorR), .temp(temp),
						  .q_I(q_I), .q_Play(q_Play), .q_Left(q_Left), .q_Right(q_Right), .q_Skip(q_Skip), .q_Done(q_Done));
						  

// OUTPUT: LEDS
	
	assign {Ld7, Ld6, Ld5, Ld4, Ld3, Ld2, Ld1, Ld0} = {q_I, q_Play, q_Left, q_Right, q_Skip, q_Done, temp}; // Reset is driven by BtnC

// SSD (Seven Segment Display)
	
	assign SSD3 = 0;
	assign SSD2 = 0;
	assign SSD1 = {1'b0, score[6:4]};
	assign SSD0 =score[3:0];
	
	assign ssdscan_clk = DIV_CLK[19:18];
	
	assign An3	= !(~(ssdscan_clk[1]) && ~(ssdscan_clk[0]));  // when ssdscan_clk = 00
	assign An2	= !(~(ssdscan_clk[1]) &&  (ssdscan_clk[0]));  // when ssdscan_clk = 01
	assign An1	=  !((ssdscan_clk[1]) && ~(ssdscan_clk[0]));  // when ssdscan_clk = 10
	assign An0	=  !((ssdscan_clk[1]) &&  (ssdscan_clk[0]));  // when ssdscan_clk = 11
	
	
	always @ (ssdscan_clk, SSD0, SSD1, SSD2, SSD3)
	begin : SSD_SCAN_OUT
		case (ssdscan_clk) 
				  2'b00: SSD = SSD3;
				  2'b01: SSD = SSD2;  	
				  2'b10: SSD = SSD1;
				  2'b11: SSD = SSD0;
		endcase 
	end
	
	assign {Ca, Cb, Cc, Cd, Ce, Cf, Cg, Dp} = {SSD_CATHODES};

	// Following is Hex-to-SSD conversion
	always @ (SSD) 
	begin : HEX_TO_SSD
		case (SSD)
			4'b0000: SSD_CATHODES = 8'b00000011; // 0
			4'b0001: SSD_CATHODES = 8'b10011111; // 1
			4'b0010: SSD_CATHODES = 8'b00100101; // 2
			4'b0011: SSD_CATHODES = 8'b00001101; // 3
			4'b0100: SSD_CATHODES = 8'b10011001; // 4
			4'b0101: SSD_CATHODES = 8'b01001001; // 5
			4'b0110: SSD_CATHODES = 8'b01000001; // 6
			4'b0111: SSD_CATHODES = 8'b00011111; // 7
			4'b1000: SSD_CATHODES = 8'b00000001; // 8
			4'b1001: SSD_CATHODES = 8'b00001001; // 9
			4'b1010: SSD_CATHODES = 8'b00010001; // A
			4'b1011: SSD_CATHODES = 8'b11000001; // B
			4'b1100: SSD_CATHODES = 8'b01100011; // C
			4'b1101: SSD_CATHODES = 8'b10000101; // D
			4'b1110: SSD_CATHODES = 8'b01100001; // E
			4'b1111: SSD_CATHODES = 8'b01110001; // F    
			default: SSD_CATHODES = 8'bXXXXXXXX; // 
		endcase
	end	
	
	//////////////////////////////////////////////////////////////////////////////////////////
	
	assign	clk = DIV_CLK[1];
	
	wire inDisplayArea;
	wire [9:0] CounterX;
	wire [9:0] CounterY;

	hvsync_generator syncgen(.clk(clk), .reset(reset),.vga_h_sync(vga_h_sync), .vga_v_sync(vga_v_sync), .inDisplayArea(inDisplayArea), .CounterX(CounterX), .CounterY(CounterY));
	
	
		/////////////////////////////////////////////////////////////////
	///////////////		VGA control starts here		/////////////////
	/////////////////////////////////////////////////////////////////
	
//****THE REDS***//	
// THE LEFT SIDE //	
wire L1 = CounterX>=33 && CounterX<=62 && CounterY>=150 && CounterY<=179;
wire L2 = CounterX>=67 && CounterX<=96 && CounterY>=150 && CounterY<=179;
wire L3 = CounterX>=101 && CounterX<=130 && CounterY>=150 && CounterY<=179;
wire L4 = CounterX>=135 && CounterX<=164 && CounterY>=150 && CounterY<=179;
wire L5 = CounterX>=169 && CounterX<=198 && CounterY>=150 && CounterY<=179;
wire L6 = CounterX>=203 && CounterX<=232 && CounterY>=150 && CounterY<=179;
wire L7 = CounterX>=237 && CounterX<=266 && CounterY>=150 && CounterY<=179;
wire L8 = CounterX>=271 && CounterX<=300 && CounterY>=150 && CounterY<=179;

wire L9 = CounterX>=33 && CounterX<=62 && CounterY>=184 && CounterY<=213;
wire L10 = CounterX>=67 && CounterX<=96 && CounterY>=184 && CounterY<=213;
wire L11 = CounterX>=101 && CounterX<=130 && CounterY>=184 && CounterY<=213;
wire L12 = CounterX>=135 && CounterX<=164 && CounterY>=184 && CounterY<=213;
wire L13 = CounterX>=169 && CounterX<=198 && CounterY>=184 && CounterY<=213;
wire L14 = CounterX>=203 && CounterX<=232 && CounterY>=184 && CounterY<=213;
wire L15 = CounterX>=237 && CounterX<=266 && CounterY>=184 && CounterY<=213;
wire L16 = CounterX>=271 && CounterX<=300 && CounterY>=184 && CounterY<=213;

wire L17 = CounterX>=33 && CounterX<=62 && CounterY>=218 && CounterY<=247;
wire L18 = CounterX>=67 && CounterX<=96 && CounterY>=218 && CounterY<=247;
wire L19 = CounterX>=101 && CounterX<=130 && CounterY>=218 && CounterY<=247;
wire L20 = CounterX>=135 && CounterX<=164 && CounterY>=218 && CounterY<=247;
wire L21 = CounterX>=169 && CounterX<=198 && CounterY>=218 && CounterY<=247;
wire L22 = CounterX>=203 && CounterX<=232 && CounterY>=218 && CounterY<=247;
wire L23 = CounterX>=237 && CounterX<=266 && CounterY>=218 && CounterY<=247;
wire L24 = CounterX>=271 && CounterX<=300 && CounterY>=218 && CounterY<=247;

wire L25 = CounterX>=33 && CounterX<=62 && CounterY>=252 && CounterY<=281;
wire L26 = CounterX>=67 && CounterX<=96 && CounterY>=252 && CounterY<=281;
wire L27 = CounterX>=101 && CounterX<=130 && CounterY>=252 && CounterY<=281;
wire L28 = CounterX>=135 && CounterX<=164 && CounterY>=252 && CounterY<=281;
wire L29 = CounterX>=169 && CounterX<=198 && CounterY>=252 && CounterY<=281;
wire L30 = CounterX>=203 && CounterX<=232 && CounterY>=252 && CounterY<=281;
wire L31 = CounterX>=237 && CounterX<=266 && CounterY>=252 && CounterY<=281;
wire L32 = CounterX>=271 && CounterX<=300 && CounterY>=252 && CounterY<=281;

wire L33 = CounterX>=33 && CounterX<=62 && CounterY>=286 && CounterY<=315;
wire L34 = CounterX>=67 && CounterX<=96 && CounterY>=286 && CounterY<=315;
wire L35 = CounterX>=101 && CounterX<=130 && CounterY>=286 && CounterY<=315;
wire L36 = CounterX>=135 && CounterX<=164 && CounterY>=286 && CounterY<=315;
wire L37 = CounterX>=169 && CounterX<=198 && CounterY>=286 && CounterY<=315;
wire L38 = CounterX>=203 && CounterX<=232 && CounterY>=286 && CounterY<=315;
wire L39 = CounterX>=237 && CounterX<=266 && CounterY>=286 && CounterY<=315;
wire L40 = CounterX>=271 && CounterX<=300 && CounterY>=286 && CounterY<=315;

wire L41 = CounterX>=33 && CounterX<=62 && CounterY>=320 && CounterY<=349;
wire L42 = CounterX>=67 && CounterX<=96 && CounterY>=320 && CounterY<=349;
wire L43 = CounterX>=101 && CounterX<=130 && CounterY>=320 && CounterY<=349;
wire L44 = CounterX>=135 && CounterX<=164 && CounterY>=320 && CounterY<=349;
wire L45 = CounterX>=169 && CounterX<=198 && CounterY>=320 && CounterY<=349;
wire L46 = CounterX>=203 && CounterX<=232 && CounterY>=320 && CounterY<=349;
wire L47 = CounterX>=237 && CounterX<=266 && CounterY>=320 && CounterY<=349;
wire L48 = CounterX>=271 && CounterX<=300 && CounterY>=320 && CounterY<=349;

wire L49 = CounterX>=33 && CounterX<=62 && CounterY>=354 && CounterY<=383;
wire L50 = CounterX>=67 && CounterX<=96 && CounterY>=354 && CounterY<=383;
wire L51 = CounterX>=101 && CounterX<=130 && CounterY>=354 && CounterY<=383;
wire L52 = CounterX>=135 && CounterX<=164 && CounterY>=354 && CounterY<=383;
wire L53 = CounterX>=169 && CounterX<=198 && CounterY>=354 && CounterY<=383;
wire L54 = CounterX>=203 && CounterX<=232 && CounterY>=354 && CounterY<=383;
wire L55 = CounterX>=237 && CounterX<=266 && CounterY>=354 && CounterY<=383;
wire L56 = CounterX>=271 && CounterX<=300 && CounterY>=354 && CounterY<=383;

wire L57 = CounterX>=33 && CounterX<=62 && CounterY>=388 && CounterY<=417;
wire L58 = CounterX>=67 && CounterX<=96 && CounterY>=388 && CounterY<=417;
wire L59 = CounterX>=101 && CounterX<=130 && CounterY>=388 && CounterY<=417;
wire L60 = CounterX>=135 && CounterX<=164 && CounterY>=388 && CounterY<=417;
wire L61 = CounterX>=169 && CounterX<=198 && CounterY>=388 && CounterY<=417;
wire L62 = CounterX>=203 && CounterX<=232 && CounterY>=388 && CounterY<=417;
wire L63 = CounterX>=237 && CounterX<=266 && CounterY>=388 && CounterY<=417;
wire L64 = CounterX>=271 && CounterX<=300 && CounterY>=388 && CounterY<=417;

//THE RIGHT SIDE//
wire R1 = CounterX>=340 && CounterX<=369 && CounterY>=150 && CounterY<=179;
wire R2 = CounterX>=374 && CounterX<=403 && CounterY>=150 && CounterY<=179;
wire R3 = CounterX>=408 && CounterX<=437 && CounterY>=150 && CounterY<=179;
wire R4 = CounterX>=442 && CounterX<=471 && CounterY>=150 && CounterY<=179;
wire R5 = CounterX>=476 && CounterX<=505 && CounterY>=150 && CounterY<=179;
wire R6 = CounterX>=510 && CounterX<=539 && CounterY>=150 && CounterY<=179;
wire R7 = CounterX>=544 && CounterX<=573 && CounterY>=150 && CounterY<=179;
wire R8 = CounterX>=578 && CounterX<=607 && CounterY>=150 && CounterY<=179;

wire R9 = CounterX>=340 && CounterX<=369 && CounterY>=184 && CounterY<=213;
wire R10 = CounterX>=374 && CounterX<=403 && CounterY>=184 && CounterY<=213;
wire R11 = CounterX>=408 && CounterX<=437 && CounterY>=184 && CounterY<=213;
wire R12 = CounterX>=442 && CounterX<=471 && CounterY>=184 && CounterY<=213;
wire R13 = CounterX>=476 && CounterX<=505 && CounterY>=184 && CounterY<=213;
wire R14 = CounterX>=510 && CounterX<=539 && CounterY>=184 && CounterY<=213;
wire R15 = CounterX>=544 && CounterX<=573 && CounterY>=184 && CounterY<=213;
wire R16 = CounterX>=578 && CounterX<=607 && CounterY>=184 && CounterY<=213;

wire R17 = CounterX>=340 && CounterX<=369 && CounterY>=218 && CounterY<=247;
wire R18 = CounterX>=374 && CounterX<=403 && CounterY>=218 && CounterY<=247;
wire R19 = CounterX>=408 && CounterX<=437 && CounterY>=218 && CounterY<=247;
wire R20 = CounterX>=442 && CounterX<=471 && CounterY>=218 && CounterY<=247;
wire R21 = CounterX>=476 && CounterX<=505 && CounterY>=218 && CounterY<=247;
wire R22 = CounterX>=510 && CounterX<=539 && CounterY>=218 && CounterY<=247;
wire R23 = CounterX>=544 && CounterX<=573 && CounterY>=218 && CounterY<=247;
wire R24 = CounterX>=578 && CounterX<=607 && CounterY>=218 && CounterY<=247;

wire R25 = CounterX>=340 && CounterX<=369 && CounterY>=252 && CounterY<=281;
wire R26 = CounterX>=374 && CounterX<=403 && CounterY>=252 && CounterY<=281;
wire R27 = CounterX>=408 && CounterX<=437 && CounterY>=252 && CounterY<=281;
wire R28 = CounterX>=442 && CounterX<=471 && CounterY>=252 && CounterY<=281;
wire R29 = CounterX>=476 && CounterX<=505 && CounterY>=252 && CounterY<=281;
wire R30 = CounterX>=510 && CounterX<=539 && CounterY>=252 && CounterY<=281;
wire R31 = CounterX>=544 && CounterX<=573 && CounterY>=252 && CounterY<=281;
wire R32 = CounterX>=578 && CounterX<=607 && CounterY>=252 && CounterY<=281;

wire R33 = CounterX>=340 && CounterX<=369 && CounterY>=286 && CounterY<=315;
wire R34 = CounterX>=374 && CounterX<=403 && CounterY>=286 && CounterY<=315;
wire R35 = CounterX>=408 && CounterX<=437 && CounterY>=286 && CounterY<=315;
wire R36 = CounterX>=442 && CounterX<=471 && CounterY>=286 && CounterY<=315;
wire R37 = CounterX>=476 && CounterX<=505 && CounterY>=286 && CounterY<=315;
wire R38 = CounterX>=510 && CounterX<=539 && CounterY>=286 && CounterY<=315;
wire R39 = CounterX>=544 && CounterX<=573 && CounterY>=286 && CounterY<=315;
wire R40 = CounterX>=578 && CounterX<=607 && CounterY>=286 && CounterY<=315;

wire R41 = CounterX>=340 && CounterX<=369 && CounterY>=320 && CounterY<=349;
wire R42 = CounterX>=374 && CounterX<=403 && CounterY>=320 && CounterY<=349;
wire R43 = CounterX>=408 && CounterX<=437 && CounterY>=320 && CounterY<=349;
wire R44 = CounterX>=442 && CounterX<=471 && CounterY>=320 && CounterY<=349;
wire R45 = CounterX>=476 && CounterX<=505 && CounterY>=320 && CounterY<=349;
wire R46 = CounterX>=510 && CounterX<=539 && CounterY>=320 && CounterY<=349;
wire R47 = CounterX>=544 && CounterX<=573 && CounterY>=320 && CounterY<=349;
wire R48 = CounterX>=578 && CounterX<=607 && CounterY>=320 && CounterY<=349;

wire R49 = CounterX>=340 && CounterX<=369 && CounterY>=354 && CounterY<=383;
wire R50 = CounterX>=374 && CounterX<=403 && CounterY>=354 && CounterY<=383;
wire R51 = CounterX>=408 && CounterX<=437 && CounterY>=354 && CounterY<=383;
wire R52 = CounterX>=442 && CounterX<=471 && CounterY>=354 && CounterY<=383;
wire R53 = CounterX>=476 && CounterX<=505 && CounterY>=354 && CounterY<=383;
wire R54 = CounterX>=510 && CounterX<=539 && CounterY>=354 && CounterY<=383;
wire R55 = CounterX>=544 && CounterX<=573 && CounterY>=354 && CounterY<=383;
wire R56 = CounterX>=578 && CounterX<=607 && CounterY>=354 && CounterY<=383;

wire R57 = CounterX>=340 && CounterX<=369 && CounterY>=388 && CounterY<=417;
wire R58 = CounterX>=374 && CounterX<=403 && CounterY>=388 && CounterY<=417;
wire R59 = CounterX>=408 && CounterX<=437 && CounterY>=388 && CounterY<=417;
wire R60 = CounterX>=442 && CounterX<=471 && CounterY>=388 && CounterY<=417;
wire R61 = CounterX>=476 && CounterX<=505 && CounterY>=388 && CounterY<=417;
wire R62 = CounterX>=510 && CounterX<=539 && CounterY>=388 && CounterY<=417;
wire R63 = CounterX>=544 && CounterX<=573 && CounterY>=388 && CounterY<=417;
wire R64 = CounterX>=578 && CounterX<=607 && CounterY>=388 && CounterY<=417;

wire Center = CounterX>=305 && CounterX<=334 && CounterY>=60 && CounterY<=89;



	always @(posedge clk)
	begin
	
/*	BLUE VERSION
	vga_r <= (	(Center & temp) |
		(L1 & colorL[0] & lightL[0]) | 
		(L2 & colorL[1] & lightL[1]) | 
		(L3 & colorL[2] & lightL[2]) | 
		(L4 & colorL[3] & lightL[3]) | 
		(L5 & colorL[4] & lightL[4]) | 
		(L6 & colorL[5] & lightL[5]) | 
		(L7 & colorL[6] & lightL[6]) | 
		(L8 & colorL[7] & lightL[7]) | 
		(L9 & colorL[8] & lightL[8]) | 
		(L10 & colorL[9] & lightL[9]) | 
		(L11 & colorL[10] & lightL[10]) | 
		(L12 & colorL[11] & lightL[11]) | 
		(L13 & colorL[12] & lightL[12]) | 
		(L14 & colorL[13] & lightL[13]) | 
		(L15 & colorL[14] & lightL[14]) | 
		(L16 & colorL[15] & lightL[15]) | 
		(L17 & colorL[16] & lightL[16]) | 
		(L18 & colorL[17] & lightL[17]) | 
		(L19 & colorL[18] & lightL[18]) | 
		(L20 & colorL[19] & lightL[19]) | 
		(L21 & colorL[20] & lightL[20]) | 
		(L22 & colorL[21] & lightL[21]) | 
		(L23 & colorL[22] & lightL[22]) | 
		(L24 & colorL[23] & lightL[23]) | 
		(L25 & colorL[24] & lightL[24]) | 
		(L26 & colorL[25] & lightL[25]) | 
		(L27 & colorL[26] & lightL[26]) | 
		(L28 & colorL[27] & lightL[27]) | 
		(L29 & colorL[28] & lightL[28]) | 
		(L30 & colorL[29] & lightL[29]) | 
		(L31 & colorL[30] & lightL[30]) | 
		(L32 & colorL[31] & lightL[31]) | 
		(L33 & colorL[32] & lightL[32]) | 
		(L34 & colorL[33] & lightL[33]) | 
		(L35 & colorL[34] & lightL[34]) | 
		(L36 & colorL[35] & lightL[35]) | 
		(L37 & colorL[36] & lightL[36]) | 
		(L38 & colorL[37] & lightL[37]) | 
		(L39 & colorL[38] & lightL[38]) | 
		(L40 & colorL[39] & lightL[39]) | 
		(L41 & colorL[40] & lightL[40]) | 
		(L42 & colorL[41] & lightL[41]) | 
		(L43 & colorL[42] & lightL[42]) | 
		(L44 & colorL[43] & lightL[43]) | 
		(L45 & colorL[44] & lightL[44]) | 
		(L46 & colorL[45] & lightL[45]) | 
		(L47 & colorL[46] & lightL[46]) | 
		(L48 & colorL[47] & lightL[47]) | 
		(L49 & colorL[48] & lightL[48]) | 
		(L50 & colorL[49] & lightL[49]) | 
		(L51 & colorL[50] & lightL[50]) | 
		(L52 & colorL[51] & lightL[51]) | 
		(L53 & colorL[52] & lightL[52]) | 
		(L54 & colorL[53] & lightL[53]) | 
		(L55 & colorL[54] & lightL[54]) | 
		(L56 & colorL[55] & lightL[55]) | 
		(L57 & colorL[56] & lightL[56]) | 
		(L58 & colorL[57] & lightL[57]) | 
		(L59 & colorL[58] & lightL[58]) | 
		(L60 & colorL[59] & lightL[59]) | 
		(L61 & colorL[60] & lightL[60]) | 
		(L62 & colorL[61] & lightL[61]) | 
		(L63 & colorL[62] & lightL[62]) | 
		(L64 & colorL[63] & lightL[63]) | 
		(R1 & colorR[0] & lightR[0]) | 
		(R2 & colorR[1] & lightR[1]) | 
		(R3 & colorR[2] & lightR[2]) | 
		(R4 & colorR[3] & lightR[3]) | 
		(R5 & colorR[4] & lightR[4]) | 
		(R6 & colorR[5] & lightR[5]) | 
		(R7 & colorR[6] & lightR[6]) | 
		(R8 & colorR[7] & lightR[7]) | 
		(R9 & colorR[8] & lightR[8]) | 
		(R10 & colorR[9] & lightR[9]) | 
		(R11 & colorR[10] & lightR[10]) | 
		(R12 & colorR[11] & lightR[11]) | 
		(R13 & colorR[12] & lightR[12]) | 
		(R14 & colorR[13] & lightR[13]) | 
		(R15 & colorR[14] & lightR[14]) | 
		(R16 & colorR[15] & lightR[15]) | 
		(R17 & colorR[16] & lightR[16]) | 
		(R18 & colorR[17] & lightR[17]) | 
		(R19 & colorR[18] & lightR[18]) | 
		(R20 & colorR[19] & lightR[19]) | 
		(R21 & colorR[20] & lightR[20]) | 
		(R22 & colorR[21] & lightR[21]) | 
		(R23 & colorR[22] & lightR[22]) | 
		(R24 & colorR[23] & lightR[23]) | 
		(R25 & colorR[24] & lightR[24]) | 
		(R26 & colorR[25] & lightR[25]) | 
		(R27 & colorR[26] & lightR[26]) | 
		(R28 & colorR[27] & lightR[27]) | 
		(R29 & colorR[28] & lightR[28]) | 
		(R30 & colorR[29] & lightR[29]) | 
		(R31 & colorR[30] & lightR[30]) | 
		(R32 & colorR[31] & lightR[31]) | 
		(R33 & colorR[32] & lightR[32]) | 
		(R34 & colorR[33] & lightR[33]) | 
		(R35 & colorR[34] & lightR[34]) | 
		(R36 & colorR[35] & lightR[35]) | 
		(R37 & colorR[36] & lightR[36]) | 
		(R38 & colorR[37] & lightR[37]) | 
		(R39 & colorR[38] & lightR[38]) | 
		(R40 & colorR[39] & lightR[39]) | 
		(R41 & colorR[40] & lightR[40]) | 
		(R42 & colorR[41] & lightR[41]) | 
		(R43 & colorR[42] & lightR[42]) | 
		(R44 & colorR[43] & lightR[43]) | 
		(R45 & colorR[44] & lightR[44]) | 
		(R46 & colorR[45] & lightR[45]) | 
		(R47 & colorR[46] & lightR[46]) | 
		(R48 & colorR[47] & lightR[47]) | 
		(R49 & colorR[48] & lightR[48]) | 
		(R50 & colorR[49] & lightR[49]) | 
		(R51 & colorR[50] & lightR[50]) | 
		(R52 & colorR[51] & lightR[51]) | 
		(R53 & colorR[52] & lightR[52]) | 
		(R54 & colorR[53] & lightR[53]) | 
		(R55 & colorR[54] & lightR[54]) | 
		(R56 & colorR[55] & lightR[55]) | 
		(R57 & colorR[56] & lightR[56]) | 
		(R58 & colorR[57] & lightR[57]) | 
		(R59 & colorR[58] & lightR[58]) | 
		(R60 & colorR[59] & lightR[59]) | 
		(R61 & colorR[60] & lightR[60]) | 
		(R62 & colorR[61] & lightR[61]) | 
		(R63 & colorR[62] & lightR[62]) | 
		(R64 & colorR[63] & lightR[63]) & inDisplayArea);
		

vga_b <= (	(Center & ~temp) |
		(L1 & ~colorL[0] & lightL[0]) | 
		(L2 & ~colorL[1] & lightL[1]) | 
		(L3 & ~colorL[2] & lightL[2]) | 
		(L4 & ~colorL[3] & lightL[3]) | 
		(L5 & ~colorL[4] & lightL[4]) | 
		(L6 & ~colorL[5] & lightL[5]) | 
		(L7 & ~colorL[6] & lightL[6]) | 
		(L8 & ~colorL[7] & lightL[7]) | 
		(L9 & ~colorL[8] & lightL[8]) | 
		(L10 & ~colorL[9] & lightL[9]) | 
		(L11 & ~colorL[10] & lightL[10]) | 
		(L12 & ~colorL[11] & lightL[11]) | 
		(L13 & ~colorL[12] & lightL[12]) | 
		(L14 & ~colorL[13] & lightL[13]) | 
		(L15 & ~colorL[14] & lightL[14]) | 
		(L16 & ~colorL[15] & lightL[15]) | 
		(L17 & ~colorL[16] & lightL[16]) | 
		(L18 & ~colorL[17] & lightL[17]) | 
		(L19 & ~colorL[18] & lightL[18]) | 
		(L20 & ~colorL[19] & lightL[19]) | 
		(L21 & ~colorL[20] & lightL[20]) | 
		(L22 & ~colorL[21] & lightL[21]) | 
		(L23 & ~colorL[22] & lightL[22]) | 
		(L24 & ~colorL[23] & lightL[23]) | 
		(L25 & ~colorL[24] & lightL[24]) | 
		(L26 & ~colorL[25] & lightL[25]) | 
		(L27 & ~colorL[26] & lightL[26]) | 
		(L28 & ~colorL[27] & lightL[27]) | 
		(L29 & ~colorL[28] & lightL[28]) | 
		(L30 & ~colorL[29] & lightL[29]) | 
		(L31 & ~colorL[30] & lightL[30]) | 
		(L32 & ~colorL[31] & lightL[31]) | 
		(L33 & ~colorL[32] & lightL[32]) | 
		(L34 & ~colorL[33] & lightL[33]) | 
		(L35 & ~colorL[34] & lightL[34]) | 
		(L36 & ~colorL[35] & lightL[35]) | 
		(L37 & ~colorL[36] & lightL[36]) | 
		(L38 & ~colorL[37] & lightL[37]) | 
		(L39 & ~colorL[38] & lightL[38]) | 
		(L40 & ~colorL[39] & lightL[39]) | 
		(L41 & ~colorL[40] & lightL[40]) | 
		(L42 & ~colorL[41] & lightL[41]) | 
		(L43 & ~colorL[42] & lightL[42]) | 
		(L44 & ~colorL[43] & lightL[43]) | 
		(L45 & ~colorL[44] & lightL[44]) | 
		(L46 & ~colorL[45] & lightL[45]) | 
		(L47 & ~colorL[46] & lightL[46]) | 
		(L48 & ~colorL[47] & lightL[47]) | 
		(L49 & ~colorL[48] & lightL[48]) | 
		(L50 & ~colorL[49] & lightL[49]) | 
		(L51 & ~colorL[50] & lightL[50]) | 
		(L52 & ~colorL[51] & lightL[51]) | 
		(L53 & ~colorL[52] & lightL[52]) | 
		(L54 & ~colorL[53] & lightL[53]) | 
		(L55 & ~colorL[54] & lightL[54]) | 
		(L56 & ~colorL[55] & lightL[55]) | 
		(L57 & ~colorL[56] & lightL[56]) | 
		(L58 & ~colorL[57] & lightL[57]) | 
		(L59 & ~colorL[58] & lightL[58]) | 
		(L60 & ~colorL[59] & lightL[59]) | 
		(L61 & ~colorL[60] & lightL[60]) | 
		(L62 & ~colorL[61] & lightL[61]) | 
		(L63 & ~colorL[62] & lightL[62]) | 
		(L64 & ~colorL[63] & lightL[63]) | 
		(R1 & ~colorR[0] & lightR[0]) | 
		(R2 & ~colorR[1] & lightR[1]) | 
		(R3 & ~colorR[2] & lightR[2]) | 
		(R4 & ~colorR[3] & lightR[3]) | 
		(R5 & ~colorR[4] & lightR[4]) | 
		(R6 & ~colorR[5] & lightR[5]) | 
		(R7 & ~colorR[6] & lightR[6]) | 
		(R8 & ~colorR[7] & lightR[7]) | 
		(R9 & ~colorR[8] & lightR[8]) | 
		(R10 & ~colorR[9] & lightR[9]) | 
		(R11 & ~colorR[10] & lightR[10]) | 
		(R12 & ~colorR[11] & lightR[11]) | 
		(R13 & ~colorR[12] & lightR[12]) | 
		(R14 & ~colorR[13] & lightR[13]) | 
		(R15 & ~colorR[14] & lightR[14]) | 
		(R16 & ~colorR[15] & lightR[15]) | 
		(R17 & ~colorR[16] & lightR[16]) | 
		(R18 & ~colorR[17] & lightR[17]) | 
		(R19 & ~colorR[18] & lightR[18]) | 
		(R20 & ~colorR[19] & lightR[19]) | 
		(R21 & ~colorR[20] & lightR[20]) | 
		(R22 & ~colorR[21] & lightR[21]) | 
		(R23 & ~colorR[22] & lightR[22]) | 
		(R24 & ~colorR[23] & lightR[23]) | 
		(R25 & ~colorR[24] & lightR[24]) | 
		(R26 & ~colorR[25] & lightR[25]) | 
		(R27 & ~colorR[26] & lightR[26]) | 
		(R28 & ~colorR[27] & lightR[27]) | 
		(R29 & ~colorR[28] & lightR[28]) | 
		(R30 & ~colorR[29] & lightR[29]) | 
		(R31 & ~colorR[30] & lightR[30]) | 
		(R32 & ~colorR[31] & lightR[31]) | 
		(R33 & ~colorR[32] & lightR[32]) | 
		(R34 & ~colorR[33] & lightR[33]) | 
		(R35 & ~colorR[34] & lightR[34]) | 
		(R36 & ~colorR[35] & lightR[35]) | 
		(R37 & ~colorR[36] & lightR[36]) | 
		(R38 & ~colorR[37] & lightR[37]) | 
		(R39 & ~colorR[38] & lightR[38]) | 
		(R40 & ~colorR[39] & lightR[39]) | 
		(R41 & ~colorR[40] & lightR[40]) | 
		(R42 & ~colorR[41] & lightR[41]) | 
		(R43 & ~colorR[42] & lightR[42]) | 
		(R44 & ~colorR[43] & lightR[43]) | 
		(R45 & ~colorR[44] & lightR[44]) | 
		(R46 & ~colorR[45] & lightR[45]) | 
		(R47 & ~colorR[46] & lightR[46]) | 
		(R48 & ~colorR[47] & lightR[47]) | 
		(R49 & ~colorR[48] & lightR[48]) | 
		(R50 & ~colorR[49] & lightR[49]) | 
		(R51 & ~colorR[50] & lightR[50]) | 
		(R52 & ~colorR[51] & lightR[51]) | 
		(R53 & ~colorR[52] & lightR[52]) | 
		(R54 & ~colorR[53] & lightR[53]) | 
		(R55 & ~colorR[54] & lightR[54]) | 
		(R56 & ~colorR[55] & lightR[55]) | 
		(R57 & ~colorR[56] & lightR[56]) | 
		(R58 & ~colorR[57] & lightR[57]) | 
		(R59 & ~colorR[58] & lightR[58]) | 
		(R60 & ~colorR[59] & lightR[59]) | 
		(R61 & ~colorR[60] & lightR[60]) | 
		(R62 & ~colorR[61] & lightR[61]) | 
		(R63 & ~colorR[62] & lightR[62]) | 
		(R64 & ~colorR[63] & lightR[63]) & inDisplayArea);
	
	vga_g <= 0; */
	
	vga_r <= (	(Center) |
		(L1 & lightL[0]) | 
		(L2 & lightL[1]) | 
		(L3 & lightL[2]) | 
		(L4 & lightL[3]) | 
		(L5 & lightL[4]) | 
		(L6 & lightL[5]) | 
		(L7 & lightL[6]) | 
		(L8 & lightL[7]) | 
		(L9 & lightL[8]) | 
		(L10 & lightL[9]) | 
		(L11 & lightL[10]) | 
		(L12 & lightL[11]) | 
		(L13 & lightL[12]) | 
		(L14 & lightL[13]) | 
		(L15 & lightL[14]) | 
		(L16 & lightL[15]) | 
		(L17 & lightL[16]) | 
		(L18 & lightL[17]) | 
		(L19 & lightL[18]) | 
		(L20 & lightL[19]) | 
		(L21 & lightL[20]) | 
		(L22 & lightL[21]) | 
		(L23 & lightL[22]) | 
		(L24 & lightL[23]) | 
		(L25 & lightL[24]) | 
		(L26 & lightL[25]) | 
		(L27 & lightL[26]) | 
		(L28 & lightL[27]) | 
		(L29 & lightL[28]) | 
		(L30 & lightL[29]) | 
		(L31 & lightL[30]) | 
		(L32 & lightL[31]) | 
		(L33 & lightL[32]) | 
		(L34 & lightL[33]) | 
		(L35 & lightL[34]) | 
		(L36 & lightL[35]) | 
		(L37 & lightL[36]) | 
		(L38 & lightL[37]) | 
		(L39 & lightL[38]) | 
		(L40 & lightL[39]) | 
		(L41 & lightL[40]) | 
		(L42 & lightL[41]) | 
		(L43 & lightL[42]) | 
		(L44 & lightL[43]) | 
		(L45 & lightL[44]) | 
		(L46 & lightL[45]) | 
		(L47 & lightL[46]) | 
		(L48 & lightL[47]) | 
		(L49 & lightL[48]) | 
		(L50 & lightL[49]) | 
		(L51 & lightL[50]) | 
		(L52 & lightL[51]) | 
		(L53 & lightL[52]) | 
		(L54 & lightL[53]) | 
		(L55 & lightL[54]) | 
		(L56 & lightL[55]) | 
		(L57 & lightL[56]) | 
		(L58 & lightL[57]) | 
		(L59 & lightL[58]) | 
		(L60 & lightL[59]) | 
		(L61 & lightL[60]) | 
		(L62 & lightL[61]) | 
		(L63 & lightL[62]) | 
		(L64 & lightL[63]) | 
		(R1 & lightR[0]) | 
		(R2 & lightR[1]) | 
		(R3 & lightR[2]) | 
		(R4 & lightR[3]) | 
		(R5 & lightR[4]) | 
		(R6 & lightR[5]) | 
		(R7 & lightR[6]) | 
		(R8 & lightR[7]) | 
		(R9 & lightR[8]) | 
		(R10 & lightR[9]) | 
		(R11 & lightR[10]) | 
		(R12 & lightR[11]) | 
		(R13 & lightR[12]) | 
		(R14 & lightR[13]) | 
		(R15 & lightR[14]) | 
		(R16 & lightR[15]) | 
		(R17 & lightR[16]) | 
		(R18 & lightR[17]) | 
		(R19 & lightR[18]) | 
		(R20 & lightR[19]) | 
		(R21 & lightR[20]) | 
		(R22 & lightR[21]) | 
		(R23 & lightR[22]) | 
		(R24 & lightR[23]) | 
		(R25 & lightR[24]) | 
		(R26 & lightR[25]) | 
		(R27 & lightR[26]) | 
		(R28 & lightR[27]) | 
		(R29 & lightR[28]) | 
		(R30 & lightR[29]) | 
		(R31 & lightR[30]) | 
		(R32 & lightR[31]) | 
		(R33 & lightR[32]) | 
		(R34 & lightR[33]) | 
		(R35 & lightR[34]) | 
		(R36 & lightR[35]) | 
		(R37 & lightR[36]) | 
		(R38 & lightR[37]) | 
		(R39 & lightR[38]) | 
		(R40 & lightR[39]) | 
		(R41 & lightR[40]) | 
		(R42 & lightR[41]) | 
		(R43 & lightR[42]) | 
		(R44 & lightR[43]) | 
		(R45 & lightR[44]) | 
		(R46 & lightR[45]) | 
		(R47 & lightR[46]) | 
		(R48 & lightR[47]) | 
		(R49 & lightR[48]) | 
		(R50 & lightR[49]) | 
		(R51 & lightR[50]) | 
		(R52 & lightR[51]) | 
		(R53 & lightR[52]) | 
		(R54 & lightR[53]) | 
		(R55 & lightR[54]) | 
		(R56 & lightR[55]) | 
		(R57 & lightR[56]) | 
		(R58 & lightR[57]) | 
		(R59 & lightR[58]) | 
		(R60 & lightR[59]) | 
		(R61 & lightR[60]) | 
		(R62 & lightR[61]) | 
		(R63 & lightR[62]) | 
		(R64 & lightR[63]) & inDisplayArea);

vga_g <= (	(Center & ~temp) |
		(L1 & ~colorL[0] & lightL[0]) | 
		(L2 & ~colorL[1] & lightL[1]) | 
		(L3 & ~colorL[2] & lightL[2]) | 
		(L4 & ~colorL[3] & lightL[3]) | 
		(L5 & ~colorL[4] & lightL[4]) | 
		(L6 & ~colorL[5] & lightL[5]) | 
		(L7 & ~colorL[6] & lightL[6]) | 
		(L8 & ~colorL[7] & lightL[7]) | 
		(L9 & ~colorL[8] & lightL[8]) | 
		(L10 & ~colorL[9] & lightL[9]) | 
		(L11 & ~colorL[10] & lightL[10]) | 
		(L12 & ~colorL[11] & lightL[11]) | 
		(L13 & ~colorL[12] & lightL[12]) | 
		(L14 & ~colorL[13] & lightL[13]) | 
		(L15 & ~colorL[14] & lightL[14]) | 
		(L16 & ~colorL[15] & lightL[15]) | 
		(L17 & ~colorL[16] & lightL[16]) | 
		(L18 & ~colorL[17] & lightL[17]) | 
		(L19 & ~colorL[18] & lightL[18]) | 
		(L20 & ~colorL[19] & lightL[19]) | 
		(L21 & ~colorL[20] & lightL[20]) | 
		(L22 & ~colorL[21] & lightL[21]) | 
		(L23 & ~colorL[22] & lightL[22]) | 
		(L24 & ~colorL[23] & lightL[23]) | 
		(L25 & ~colorL[24] & lightL[24]) | 
		(L26 & ~colorL[25] & lightL[25]) | 
		(L27 & ~colorL[26] & lightL[26]) | 
		(L28 & ~colorL[27] & lightL[27]) | 
		(L29 & ~colorL[28] & lightL[28]) | 
		(L30 & ~colorL[29] & lightL[29]) | 
		(L31 & ~colorL[30] & lightL[30]) | 
		(L32 & ~colorL[31] & lightL[31]) | 
		(L33 & ~colorL[32] & lightL[32]) | 
		(L34 & ~colorL[33] & lightL[33]) | 
		(L35 & ~colorL[34] & lightL[34]) | 
		(L36 & ~colorL[35] & lightL[35]) | 
		(L37 & ~colorL[36] & lightL[36]) | 
		(L38 & ~colorL[37] & lightL[37]) | 
		(L39 & ~colorL[38] & lightL[38]) | 
		(L40 & ~colorL[39] & lightL[39]) | 
		(L41 & ~colorL[40] & lightL[40]) | 
		(L42 & ~colorL[41] & lightL[41]) | 
		(L43 & ~colorL[42] & lightL[42]) | 
		(L44 & ~colorL[43] & lightL[43]) | 
		(L45 & ~colorL[44] & lightL[44]) | 
		(L46 & ~colorL[45] & lightL[45]) | 
		(L47 & ~colorL[46] & lightL[46]) | 
		(L48 & ~colorL[47] & lightL[47]) | 
		(L49 & ~colorL[48] & lightL[48]) | 
		(L50 & ~colorL[49] & lightL[49]) | 
		(L51 & ~colorL[50] & lightL[50]) | 
		(L52 & ~colorL[51] & lightL[51]) | 
		(L53 & ~colorL[52] & lightL[52]) | 
		(L54 & ~colorL[53] & lightL[53]) | 
		(L55 & ~colorL[54] & lightL[54]) | 
		(L56 & ~colorL[55] & lightL[55]) | 
		(L57 & ~colorL[56] & lightL[56]) | 
		(L58 & ~colorL[57] & lightL[57]) | 
		(L59 & ~colorL[58] & lightL[58]) | 
		(L60 & ~colorL[59] & lightL[59]) | 
		(L61 & ~colorL[60] & lightL[60]) | 
		(L62 & ~colorL[61] & lightL[61]) | 
		(L63 & ~colorL[62] & lightL[62]) | 
		(L64 & ~colorL[63] & lightL[63]) | 
		(R1 & ~colorR[0] & lightR[0]) | 
		(R2 & ~colorR[1] & lightR[1]) | 
		(R3 & ~colorR[2] & lightR[2]) | 
		(R4 & ~colorR[3] & lightR[3]) | 
		(R5 & ~colorR[4] & lightR[4]) | 
		(R6 & ~colorR[5] & lightR[5]) | 
		(R7 & ~colorR[6] & lightR[6]) | 
		(R8 & ~colorR[7] & lightR[7]) | 
		(R9 & ~colorR[8] & lightR[8]) | 
		(R10 & ~colorR[9] & lightR[9]) | 
		(R11 & ~colorR[10] & lightR[10]) | 
		(R12 & ~colorR[11] & lightR[11]) | 
		(R13 & ~colorR[12] & lightR[12]) | 
		(R14 & ~colorR[13] & lightR[13]) | 
		(R15 & ~colorR[14] & lightR[14]) | 
		(R16 & ~colorR[15] & lightR[15]) | 
		(R17 & ~colorR[16] & lightR[16]) | 
		(R18 & ~colorR[17] & lightR[17]) | 
		(R19 & ~colorR[18] & lightR[18]) | 
		(R20 & ~colorR[19] & lightR[19]) | 
		(R21 & ~colorR[20] & lightR[20]) | 
		(R22 & ~colorR[21] & lightR[21]) | 
		(R23 & ~colorR[22] & lightR[22]) | 
		(R24 & ~colorR[23] & lightR[23]) | 
		(R25 & ~colorR[24] & lightR[24]) | 
		(R26 & ~colorR[25] & lightR[25]) | 
		(R27 & ~colorR[26] & lightR[26]) | 
		(R28 & ~colorR[27] & lightR[27]) | 
		(R29 & ~colorR[28] & lightR[28]) | 
		(R30 & ~colorR[29] & lightR[29]) | 
		(R31 & ~colorR[30] & lightR[30]) | 
		(R32 & ~colorR[31] & lightR[31]) | 
		(R33 & ~colorR[32] & lightR[32]) | 
		(R34 & ~colorR[33] & lightR[33]) | 
		(R35 & ~colorR[34] & lightR[34]) | 
		(R36 & ~colorR[35] & lightR[35]) | 
		(R37 & ~colorR[36] & lightR[36]) | 
		(R38 & ~colorR[37] & lightR[37]) | 
		(R39 & ~colorR[38] & lightR[38]) | 
		(R40 & ~colorR[39] & lightR[39]) | 
		(R41 & ~colorR[40] & lightR[40]) | 
		(R42 & ~colorR[41] & lightR[41]) | 
		(R43 & ~colorR[42] & lightR[42]) | 
		(R44 & ~colorR[43] & lightR[43]) | 
		(R45 & ~colorR[44] & lightR[44]) | 
		(R46 & ~colorR[45] & lightR[45]) | 
		(R47 & ~colorR[46] & lightR[46]) | 
		(R48 & ~colorR[47] & lightR[47]) | 
		(R49 & ~colorR[48] & lightR[48]) | 
		(R50 & ~colorR[49] & lightR[49]) | 
		(R51 & ~colorR[50] & lightR[50]) | 
		(R52 & ~colorR[51] & lightR[51]) | 
		(R53 & ~colorR[52] & lightR[52]) | 
		(R54 & ~colorR[53] & lightR[53]) | 
		(R55 & ~colorR[54] & lightR[54]) | 
		(R56 & ~colorR[55] & lightR[55]) | 
		(R57 & ~colorR[56] & lightR[56]) | 
		(R58 & ~colorR[57] & lightR[57]) | 
		(R59 & ~colorR[58] & lightR[58]) | 
		(R60 & ~colorR[59] & lightR[59]) | 
		(R61 & ~colorR[60] & lightR[60]) | 
		(R62 & ~colorR[61] & lightR[61]) | 
		(R63 & ~colorR[62] & lightR[62]) | 
		(R64 & ~colorR[63] & lightR[63]) & inDisplayArea);
	
	vga_b <= 0;	
		
	end
	
	
endmodule
