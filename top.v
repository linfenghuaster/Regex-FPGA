module top(input size, input_char, clk, reset,
             output input_char_flag);

	wire [19:0] tb_addr;
	wire [31:0] tb_rd_bus;
	
	wire        input_char_flag;
	
	
	design_1_wrapper B_Mem
	   (.BRAM_PORTA_addr(tb_addr),
		.BRAM_PORTA_clk(clk),
		.BRAM_PORTA_dout(tb_rd_bus));
		
	CSR_traversal C1 
		(.clk(clk),
		.reset(reset),
		.size(size), 
		.rd_address(tb_addr), 
		.rd_bus(tb_rd_bus), 
		.input_char_flag(input_char_flag),
		.input_char(input_char));
		
endmodule