`timescale 1 ns/1 ps

module Blk_Mem_tb;
   
	reg [7:0] 	input_char;
	reg 		input_char_flag;
	
	reg tb_clk = 1;
	wire [4095:0] tb_rd_bus;
	wire [16:0] tb_addr;
	reg 		reset;
	reg [23:0]	size;
	reg [7:0]	data_read [500000:0];
	integer m = 0, i, h;
	integer cycles = 0;
	
    always  #5  tb_clk = ~tb_clk;
	
	initial
	begin
	
		#2
		reset = 1;
		h = 1;
		$readmemh("input_trace.txt",data_read);
		
		#10
		reset = 0;
		size = 7;

	end
	
	always @(posedge tb_clk)
	begin
		#1
		cycles = cycles + 1;
		if(input_char_flag == 1)
		begin
			#1
			input_char = data_read[m];
			m = m + 1;
		end
		
		if(reset == 0 && m == 9570)
		begin	
			
			#20;
			reset = 1;
			$display($time,"\nTotal no. cycles: %d", cycles);
			$finish;
		end
	end
	
	design_1_wrapper B_Mem
	   (.BRAM_PORTA_addr(tb_addr),
		.BRAM_PORTA_clk(tb_clk),
		.BRAM_PORTA_dout(tb_rd_bus));
		
	CSR_traversal C1 
		(.clk(tb_clk),
		.reset(reset),
		.size(size), 
		.rd_address(tb_addr), 
		.rd_bus(tb_rd_bus), 
		.input_char_flag(input_char_flag),
		.input_char(input_char));
	
endmodule