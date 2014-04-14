`timescale 1ns / 1ps

module clock(Clk, Reset, Pulse, N);	

	// parameters
	parameter WIDTH = 28;


	/*  INPUTS */
	// Clock & Reset
	input		Clk, Reset;
	// Terminal count
	input [WIDTH-1:0] N;	

	reg [WIDTH-1:0] Nlast;	

	
	reg [WIDTH-1:0] Count;	

	
	/*  OUTPUTS */
	// output clock
	output Pulse;

	assign Pulse = (Count == N-1) && (N!=0); 

		
	always @ (posedge Clk, posedge Reset)
	begin : PULSE_GENERATOR
		if(Reset)
			Count <= 0;
		else
			begin

    			Nlast <= N;
		    	if(Nlast!=N)
		      		Count <= 0;
        		else if(Count == N-1)
          			begin
					
						Count <= 0;

          			end
			  	else
			  		begin
						
						Count <= Count + 1'b1;

					end
		  end
	end


endmodule
