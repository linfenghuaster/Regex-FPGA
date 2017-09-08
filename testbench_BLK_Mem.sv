`timescale 1 ns/1 ps

module Blk_Mem_tb;
   
	reg [7:0] 	input_char;
	reg [7:0] 	input_char_2;
	reg 		input_char_flag;
	reg 		accepting_match_flag;
	reg 		accepting_match_flag_2;
	
	reg tb_clk = 1;
	wire [511:0] tb_rd_bus;
	wire [16:0] tb_addr;
	reg 		reset;
	reg [23:0]	size;
	reg [7:0]	data_read_lo [200000:0];
	reg [7:0]	data_read_hi [200000:0];
	int m = 0, h;
	int cycles = 0;
	parameter size_range = 9514;
	logic 	[9:0] 	match_count[size_range - 1:0];
	logic 	[9:0] 	match_count_2[size_range - 1:0];
	reg 	[19:0] 	i;
	integer fp;
	
    always  #5  tb_clk = ~tb_clk;
	
	initial
	begin
		
		#2
		reset = 1;
		h = 1;
		$readmemh("input_trace_lo.mem",data_read_lo);
		$readmemh("input_trace_hi.mem",data_read_hi);
				
		#10
		reset = 0;
		size = size_range;
		
		for(int p=0; p < size; p=p+1)
		begin
			match_count[p] = 0;
			match_count_2[p] = 0;
		end

	end
	
	always @(posedge tb_clk)
	begin
		#1
		cycles = cycles + 1;
		if(input_char_flag == 1)
		begin
			#1
			input_char = data_read_lo[m];
			input_char_2 = data_read_hi[m];
			m = m + 1;
		end
		
		if(accepting_match_flag == 1)
		begin
			match_count[i] = match_count[i] + 1;
		end
		
		if(accepting_match_flag_2 == 1)
		begin
			match_count_2[i] = match_count_2[i] + 1;
		end
		
		if(reset == 0 && m == 200000)
		begin	
					
			#20;
			foreach (match_count[p])
			    if(match_count[p] !== 0)
			    	$display("match_count[%d] = %d", p, match_count[p]);
			
			foreach (match_count_2[p])
			    if(match_count_2[p] !== 0)
			    	$display("match_count_2[%d] = %d", p, match_count_2[p]);
				
			reset = 1;
			$display($time,"\nTotal no. cycles: %d", cycles);
			$finish;
		end
	end
	
	design_1_wrapper B_Mem
	   (.BRAM_PORTA_addr(tb_addr),
		.BRAM_PORTA_clk(tb_clk),
		.BRAM_PORTA_dout(tb_rd_bus));
		
	CSR_traversal #(.size_range(size_range)) C1 
		(.clk(tb_clk),
		.reset(reset),
		.size(size), 
		.rd_address(tb_addr), 
		.rd_bus(tb_rd_bus), 
		.input_char_flag(input_char_flag),
		.input_char(input_char),
		.input_char_2(input_char_2),
		.i(i),
		.accepting_match_flag(accepting_match_flag),
		.accepting_match_flag_2(accepting_match_flag_2)
		);
	
endmodule