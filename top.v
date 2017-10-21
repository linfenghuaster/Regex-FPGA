module top	(input input_char_2, input_char, clk, reset, 
			 output input_char_flag, accepting_match_flag, 
				accepting_match_flag_2, i);

	wire [19:0] tb_addr;
	wire [31:0] tb_rd_bus;
	
	parameter size_range = 9514;
	
	design_1_wrapper B_Mem
	   (.BRAM_PORTA_addr(tb_addr),
		.BRAM_PORTA_clk(clk),
		.BRAM_PORTA_dout(tb_rd_bus));
		
	CSR_traversal #(.size_range(size_range)) C1 
		(.clk(tb_clk),
		.reset(reset),
		.size(size_range), 
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