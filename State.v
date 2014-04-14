`timescale 1ns / 1ps

module State(Clk, rand, LEFT_Btn, RIGHT_Btn, Reset, Start, Ack, Pulse, score, posL, posR, lightL, lightR, colorL, colorR, temp, q_I, q_Play, q_Left, q_Right, q_Skip, q_Done);

	/*  INPUTS */
	input	Clk, rand, Reset, Start, Ack, Pulse, LEFT_Btn, RIGHT_Btn;

	output reg [5:0] posR, posL;
   output reg [6:0] score;		
   output reg [63:0] lightR, lightL, colorR, colorL;	
   output reg temp;		

	// store current state
	output q_I, q_Play, q_Left, q_Right, q_Skip, q_Done;
	reg [5:0] state;	
	assign {q_Done, q_Skip, q_Right, q_Left, q_Play, q_I} = state;
	
	reg [1:0] wrongR, wrongL;
	
	localparam 	
	I = 6'b000001, PLAY = 6'b000010, LEFT = 6'b000100, RIGHT = 6'b001000, SKIP = 6'b010000, DONE = 6'b100000, UNK = 6'bXXXXXX;
	
	// NSL AND SM
	always @ (posedge Clk, posedge Reset)
	begin : NumeroUno
		if(Reset) 
		  begin
			state <= I;
		  end
		else				
				case(state)	
					I:
					begin
						// state transfers
						if (Start) 
						begin
						state <= PLAY;
						end
						// initializations
						posR <= 6'b000000;
						posL <= 6'b000000;
						lightR <= 64'b0;
						lightL <= 64'b0;
						colorR <= 64'b0;
						colorL <= 64'b0;	
						score <= 7'b0000000;
						wrongR <= 2'b00;
						wrongL <= 2'b00;
						temp = rand;
					end		
					PLAY: 
							begin
		               if (LEFT_Btn)
								state <=LEFT;
							else if (RIGHT_Btn)
								state <=RIGHT;
							else if (Pulse)
								state <=SKIP;
							end
					LEFT:
						begin
							// state transfers
							if ((posL==6'b111111)||(wrongL==2'b10&&temp==1))
								state <= DONE; 
							else
								state <= PLAY; 
							// data transfers							
							posL <= posL + 1;
							lightL[posL] <= 1;
							if (temp==0)
								score<=score+1;
							else
								begin
								colorL[posL]<=1;
								wrongL<=wrongL+1;
								end
							temp = rand;
						end
					RIGHT:
						begin
							// state transfers
							if ((posR==6'b111111)||(wrongR==2'b10&&temp==0))
								state <= DONE; 
							else
								state <= PLAY; 
							// data transfers							
							posR <= posR + 1;
							lightR[posR] <= 1;
							if (temp==1)
								begin
								colorR[posR]<=1;
								score<=score+1;
								end
							else
								wrongR<=wrongR+1;
							temp = rand;
						end
					SKIP:
						begin
							// state transfers
							if ((posR==6'b111111)||(posL==6'b111111))
								state <= DONE; 
							else
								state <= PLAY; 
							// data transfers							
							if (temp==0)
								posL<=posL+1;
							else
								posR<=posR+1;
							temp = rand;
						end
					DONE:
						if (Ack)	state <= I;
					default:		
						state <= UNK;
				endcase
	end
		

endmodule
