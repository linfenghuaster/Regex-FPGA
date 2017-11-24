/******************************************************
*
* 	NAME:	CSR_traversal
*
* 	DESCRIPTION:   
*	Implementing the CSR NFA traversal by 
*	traversing in BFS fashion and reading 
*	the CSR stored in the Block memory.
*  
* 	REVISION HISTORY:
*	Date		Programmer		Description/Version
*	05/03/17	Kunal Buch		Version 1.0 [Parallel Traversal]
*	05/13/17	Kunal Buch		Version 1.1 [Parallel Traversal of vector length = 8 and clean read with read.tcl]
*	05/27/17	Kunal Buch		Version 1.2 [Parallel Traversal of vector length = 16]
*	06/07/17	Kunal Buch		Version 1.3	[Parallel Traversal of vector length = 128]
*	07/15/17	Kunal Buch		Version 1.4 [Parallel Traversal of vector length = 16]
*	07/16/17	Kunal Buch		Version 1.5 [Parallel Traversal of vector length = 16 and processing 2 streams in parallel]
*	07/16/17	Kunal Buch		Version 1.6 [Parallel Traversal of vector length = 4 and processing 2 streams in parallel]
*
********************************************************/
`timescale 1 ns/1 ps

module CSR_traversal (clk, reset, size, rd_address, rd_bus, input_char_flag, input_char, input_char_2, i, accepting_match_flag, accepting_match_flag_2);

	// interface signals
	input	[23:0]		size;
	input 	[511:0]		rd_bus;
	input 				clk;
	input 				reset;
	input	[7:0]		input_char;
	input	[7:0]		input_char_2;
	
	output 	[15:0]		rd_address;
	output 				input_char_flag;
	output	[19:0]		i;
	output				accepting_match_flag;
	output				accepting_match_flag_2;

	reg 	[15:0]		rd_address;
	reg 				input_char_flag;
	reg		[19:0]		i;		// allowing upto 65k states
	reg 				accepting_match_flag;
	reg 				accepting_match_flag_2;
	
	integer iter;
	parameter size_range = 0;
	parameter parallel_lane = 4;
	
	// internal signals local to the block
	reg	[24:0]		offset;
	reg [9:0]		flag;
	reg [23:0]		first;
	// reg [9:0]		last;
	reg [size_range - 1:0]			next;
	reg [size_range - 1:0]			current;		//current and next state bitmaps for stream 1
	reg [size_range - 1:0]			next_2;
	reg [size_range - 1:0]			current_2;		//current and next state bitmaps for stream 1
	
	reg [size_range - 1:0]			accepting;		//tracking the terminal states
	
	reg	[7:0]		block_mem_state_info_transition		[parallel_lane-1:0]; 
	reg	[23:0]		block_mem_state_info_target_state	[parallel_lane-1:0];
	
	//for calculating number of transitions for each state
	reg [23:0]		range;
	reg [23:0]		range_last;
	reg [23:0]		range_int;
	reg [23:0]		up_counter;
	reg [23:0]		up_counter_int;
	
	reg [2:0]		state;
	reg [24:0]		rd_address_int;
	reg 			flag_iter_check;
	reg 			flag_check;
	
	reg 			range_2_state;
	reg 			range_1_state;
	
	// adding parallel part for caching
	
	// starting params
	parameter mem0 = 31;
	parameter mem1 = mem0 + 32;  	//63;
	parameter mem2 = mem1 + 32;  	//95;
	parameter mem3 = mem2 + 32;  	//127;	
	
	//ending params
	parameter mem1_e = 32;
	parameter mem2_e = mem1_e + 32;  	//64;
	parameter mem3_e = mem2_e + 32;  	//96;
	parameter mem4_e = mem3_e + 32;  	//128;
	
	
	reg [31:0]		cache	[parallel_lane-1:0];
	reg [31:0]		cache_temp;
	reg 			range_next;
	reg [15:0]		cache_line_no;
	reg [5:0]		current_cached;
	reg [3:0]		block_offset;
	reg [3:0]		block_offset_flag_0;
	reg [3:0]		block_offset_plus_one;
	reg [3:0]		block_offset_plus_one_reg;
	reg [3:0]		block_offset_reg;
	reg [1:0]		flag_1_or_2;
	reg [4:0]		no_cached_blocks;
	reg [4:0]		no_cached_blocks_int;
	reg [4:0]		no_cached_blocks_flag_0;
	reg [4:0]		no_cached_blocks_flag_1;
	reg [4:0]		no_cached_blocks_flag_2;
	reg [4:0]		no_cached_blocks_flag_2_prev;
	reg [1:0]		flag_2;
	
	reg check;
	
	always@(posedge clk)
	begin
	
		//Reset behaviour and initialization
		if(reset)
		begin
			
			i <= 0;
			state <= 0;
			
			input_char_flag <= 1;
			accepting_match_flag <= 0;
			accepting_match_flag_2 <= 0;
			
			range_2_state <= 0;
			range_1_state <= 0;
			range_next <= 0;
			
			
			current[size_range - 1:1] <= 0;
			current_2[size_range - 1:1] <= 0;
			next <= 0;
			next_2 <= 0;
			accepting <= 0;
			
			/*for(iter = 1; iter < size_range; iter = iter + 1)
			begin
				active_2[iter] <= 0;
				active[iter] <= 0;
			end*/
			
			current[0] <= 1;
			current_2[0] <= 1;
			//active[0] <= 1;
			//active_2[0] <= 1;
			
			range <= 0;
			flag_check <= 0;			
		end
		else
		begin
			
			//checking if the given state 'i' is currently active or not
			if((current[i] == 1 || current_2[i] == 1) && state == 0)
			begin
				input_char_flag <= 0;
				rd_address <= cache_line_no;
				block_offset_reg <= block_offset;
				block_offset_plus_one_reg <= block_offset_plus_one;
				state <= 1;
			end
			else if(state == 1)
			begin
				//for special case where the for calculation the range, the lower margin is the 16th element of the caches line
				//edit below : if(block_offset_reg == 15)
				if(block_offset_reg == 3)
				begin
					rd_address <= rd_address + 1;
				end
				state <= 2;
			end
			else if(state == 2)
			begin
				
				//edit below : if(block_offset_reg != 15)
				if(block_offset_reg != 3)
				begin
					range <= cache[block_offset_plus_one_reg] - cache[block_offset_reg];
					up_counter <= cache[block_offset_reg];
					flag <= 0;
					state <= 3;
				end
				else 
				begin
					//waiting for the newly requested additional cache block
					if(range_next == 0)
					begin
						range_next <= 1;
						//cache_temp <= cache[15];
						cache_temp <= cache[3];
					end
					else if(range_next == 1)
					begin
						range_next <= 0;
						
						range <= cache[0] - cache_temp;
						up_counter <= cache_temp;
						flag <= 0;
						state <= 3;
					end
				end
				check <= 0;
			end
			else if(state == 3)
			begin
				if(range == 0 && flag == 0)
				begin
					//noting down that the state is an accepting state.
					accepting[i] <= 1;
					
					if(current[i] == 1)
					begin
						accepting_match_flag <= 1;
					end
					
					if(current_2[i] == 1)
					begin
						accepting_match_flag_2 <= 1;
					end
					
					state <= 4;
				end
				else if(range > 0)
				begin
					if(flag == 0)
					begin
					
						//fetching the cache line with the transitions of the given state
						rd_address <= cache_line_no;
						flag_1_or_2 <= 0;
						block_offset_flag_0 <= block_offset;
						no_cached_blocks_flag_0 <= no_cached_blocks;
						//range_last <= range;
						range <= range_int;
						up_counter <= up_counter_int;
						
						flag <= 1;
					end
					else if(flag == 1)
					begin
						flag <= 2;
						rd_address <= cache_line_no;
						flag_1_or_2 <= 1;
						no_cached_blocks_flag_1 <= no_cached_blocks;
						range_last <= range;
						range <= range_int;
						up_counter <= up_counter_int;
						flag_2 <= 0;
						
					end
					else if(flag == 2)
					begin
						
						// traversing in a 16-wide parallel lane for the line fetched in flag = 0 state
						if(flag_2 == 0)
						begin
							
							if(block_offset_flag_0 == 0 && no_cached_blocks_flag_0 >= 1)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[0] == input_char)
									next[block_mem_state_info_target_state[0]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[0] == input_char_2)
									next_2[block_mem_state_info_target_state[0]] <= 1;
 							end
							
							if(	(block_offset_flag_0 == 0 && no_cached_blocks_flag_0 >= 2) ||
								(block_offset_flag_0 == 1 && no_cached_blocks_flag_0 >= 1)
							)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[1] == input_char)
									next[block_mem_state_info_target_state[1]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[1] == input_char_2)
									next_2[block_mem_state_info_target_state[1]] <= 1;
							end
							
							if(	(block_offset_flag_0 == 0 && no_cached_blocks_flag_0 >= 3) ||
								(block_offset_flag_0 == 1 && no_cached_blocks_flag_0 >= 2) ||
								(block_offset_flag_0 == 2 && no_cached_blocks_flag_0 >= 1)
							)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[2] == input_char)
									next[block_mem_state_info_target_state[2]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[2] == input_char_2)
									next_2[block_mem_state_info_target_state[2]] <= 1;
							end
							
							if(	(block_offset_flag_0 == 0 && no_cached_blocks_flag_0 >= 4) ||
								(block_offset_flag_0 == 1 && no_cached_blocks_flag_0 >= 3) ||
								(block_offset_flag_0 == 2 && no_cached_blocks_flag_0 >= 2) ||
								(block_offset_flag_0 == 3 && no_cached_blocks_flag_0 >= 1)
							)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[3] == input_char)
									next[block_mem_state_info_target_state[3]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[3] == input_char_2)
									next_2[block_mem_state_info_target_state[3]] <= 1;
							end
							
							/********* First one till 15*******/
							
							flag_2 <= 1;
						end
						// traversing in a 16-wide parallel lane for the line fetched in flag = 1 state
						else if(flag_2 <= 1)
						begin
							
							if(no_cached_blocks_flag_1 > 0)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[0] == input_char)
									next[block_mem_state_info_target_state[0]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[0] == input_char_2)
									next_2[block_mem_state_info_target_state[0]] <= 1;
							end
							
							if(no_cached_blocks_flag_1 > 1)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[1] == input_char)
									next[block_mem_state_info_target_state[1]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[1] == input_char_2)
									next_2[block_mem_state_info_target_state[1]] <= 1;
							end
							
							if(no_cached_blocks_flag_1 > 2)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[2] == input_char)
									next[block_mem_state_info_target_state[2]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[2] == input_char_2)
									next_2[block_mem_state_info_target_state[2]] <= 1;
							end
							
							if(no_cached_blocks_flag_1 > 3)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[3] == input_char)
									next[block_mem_state_info_target_state[3]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[3] == input_char_2)
									next_2[block_mem_state_info_target_state[3]] <= 1;
							end
							
							/********* First one till 15*******/
							
							flag_2 <= 2;
						end
						// traversing in a 16-wide parallel lane for the line fetched in flag = 2 state
						else if(flag_2 <= 2)
						begin
							if(no_cached_blocks_flag_2 > 0)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[0] == input_char)
									next[block_mem_state_info_target_state[0]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[0] == input_char_2)
									next_2[block_mem_state_info_target_state[0]] <= 1;
							end
							
							if(no_cached_blocks_flag_2 > 1)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[1] == input_char)
									next[block_mem_state_info_target_state[1]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[1] == input_char_2)
									next_2[block_mem_state_info_target_state[1]] <= 1;
							end
							
							if(no_cached_blocks_flag_2 > 2)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[2] == input_char)
									next[block_mem_state_info_target_state[2]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[2] == input_char_2)
									next_2[block_mem_state_info_target_state[2]] <= 1;
							end
							
							if(no_cached_blocks_flag_2 > 3)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[3] == input_char)
									next[block_mem_state_info_target_state[3]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[3] == input_char_2)
									next_2[block_mem_state_info_target_state[3]] <= 1;
							end
					
							/********* First one till 15*******/
					
						end
						
						flag_1_or_2 <= 2;
						no_cached_blocks_flag_2_prev <= no_cached_blocks_flag_2;
						no_cached_blocks_flag_2 <= no_cached_blocks;
					
						range_last <= range;
						range <= range_int;
						up_counter <= up_counter_int;
						rd_address <= cache_line_no;	
					end
				end
				else if(range == 0)
				begin
					
					if(flag == 1 && range_1_state == 0)
					begin
						range_1_state <= 1;
					end
					else if(flag == 1 && range_1_state == 1)
					begin
						// traversing in a 16-wide parallel lane for the line fetched in flag = 0 state
						if(flag_1_or_2 == 0)
						begin
					
							if(block_offset_flag_0 == 0 && no_cached_blocks_flag_0 >= 1)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[0] == input_char)
									next[block_mem_state_info_target_state[0]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[0] == input_char_2)
									next_2[block_mem_state_info_target_state[0]] <= 1;
 							end
							
							if(	(block_offset_flag_0 == 0 && no_cached_blocks_flag_0 >= 2) ||
								(block_offset_flag_0 == 1 && no_cached_blocks_flag_0 >= 1)
							)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[1] == input_char)
									next[block_mem_state_info_target_state[1]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[1] == input_char_2)
									next_2[block_mem_state_info_target_state[1]] <= 1;
							end
							
							if(	(block_offset_flag_0 == 0 && no_cached_blocks_flag_0 >= 3) ||
								(block_offset_flag_0 == 1 && no_cached_blocks_flag_0 >= 2) ||
								(block_offset_flag_0 == 2 && no_cached_blocks_flag_0 >= 1)
							)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[2] == input_char)
									next[block_mem_state_info_target_state[2]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[2] == input_char_2)
									next_2[block_mem_state_info_target_state[2]] <= 1;
							end
							
							if(	(block_offset_flag_0 == 0 && no_cached_blocks_flag_0 >= 4) ||
								(block_offset_flag_0 == 1 && no_cached_blocks_flag_0 >= 3) ||
								(block_offset_flag_0 == 2 && no_cached_blocks_flag_0 >= 2) ||
								(block_offset_flag_0 == 3 && no_cached_blocks_flag_0 >= 1)
							)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[3] == input_char)
									next[block_mem_state_info_target_state[3]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[3] == input_char_2)
									next_2[block_mem_state_info_target_state[3]] <= 1;
							end
						
							/********* First one till 15*******/
							
						end
	
						range_1_state <= 0;
						state <= 4;
						flag <= 0;
					end
					else if(flag == 2 && range_2_state == 0)
					begin
						
						// traversing in a 16-wide parallel lane for the line fetched in flag = 1 state but is compared here because the range reached 0 
						if(flag_2 == 2)
						begin
					
							if(no_cached_blocks_flag_2_prev > 0)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[0] == input_char)
									next[block_mem_state_info_target_state[0]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[0] == input_char_2)
									next_2[block_mem_state_info_target_state[0]] <= 1;
							end
							
							if(no_cached_blocks_flag_2_prev > 1)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[1] == input_char)
									next[block_mem_state_info_target_state[1]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[1] == input_char_2)
									next_2[block_mem_state_info_target_state[1]] <= 1;
							end
							
							if(no_cached_blocks_flag_2_prev > 2)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[2] == input_char)
									next[block_mem_state_info_target_state[2]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[2] == input_char_2)
									next_2[block_mem_state_info_target_state[2]] <= 1;
							end
							
							if(no_cached_blocks_flag_2_prev > 3)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[3] == input_char)
									next[block_mem_state_info_target_state[3]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[3] == input_char_2)
									next_2[block_mem_state_info_target_state[3]] <= 1;
							end
		
							/********* First one till 15*******/
						end
						// traversing in a 16-wide parallel lane for the line fetched in flag = 1 state
						if(flag_2 == 1)
						begin
							if(no_cached_blocks_flag_1 > 0)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[0] == input_char)
									next[block_mem_state_info_target_state[0]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[0] == input_char_2)
									next_2[block_mem_state_info_target_state[0]] <= 1;
							end
							
							if(no_cached_blocks_flag_1 > 1)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[1] == input_char)
									next[block_mem_state_info_target_state[1]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[1] == input_char_2)
									next_2[block_mem_state_info_target_state[1]] <= 1;
							end
							
							if(no_cached_blocks_flag_1 > 2)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[2] == input_char)
									next[block_mem_state_info_target_state[2]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[2] == input_char_2)
									next_2[block_mem_state_info_target_state[2]] <= 1;
							end
							
							if(no_cached_blocks_flag_1 > 3)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[3] == input_char)
									next[block_mem_state_info_target_state[3]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[3] == input_char_2)
									next_2[block_mem_state_info_target_state[3]] <= 1;
							end
									
							/********* First one till 15*******/
						end
						// traversing in a 16-wide parallel lane for the line fetched in flag = 0 state
						if(flag_1_or_2 == 1)
						begin
							
							if(block_offset_flag_0 == 0 && no_cached_blocks_flag_0 >= 1)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[0] == input_char)
									next[block_mem_state_info_target_state[0]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[0] == input_char_2)
									next_2[block_mem_state_info_target_state[0]] <= 1;
 							end
							
							if(	(block_offset_flag_0 == 0 && no_cached_blocks_flag_0 >= 2) ||
								(block_offset_flag_0 == 1 && no_cached_blocks_flag_0 >= 1)
							)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[1] == input_char)
									next[block_mem_state_info_target_state[1]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[1] == input_char_2)
									next_2[block_mem_state_info_target_state[1]] <= 1;
							end
							
							if(	(block_offset_flag_0 == 0 && no_cached_blocks_flag_0 >= 3) ||
								(block_offset_flag_0 == 1 && no_cached_blocks_flag_0 >= 2) ||
								(block_offset_flag_0 == 2 && no_cached_blocks_flag_0 >= 1)
							)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[2] == input_char)
									next[block_mem_state_info_target_state[2]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[2] == input_char_2)
									next_2[block_mem_state_info_target_state[2]] <= 1;
							end
							
							if(	(block_offset_flag_0 == 0 && no_cached_blocks_flag_0 >= 4) ||
								(block_offset_flag_0 == 1 && no_cached_blocks_flag_0 >= 3) ||
								(block_offset_flag_0 == 2 && no_cached_blocks_flag_0 >= 2) ||
								(block_offset_flag_0 == 3 && no_cached_blocks_flag_0 >= 1)
							)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[3] == input_char)
									next[block_mem_state_info_target_state[3]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[3] == input_char_2)
									next_2[block_mem_state_info_target_state[3]] <= 1;
							end
						
							/********* First one till 15*******/
						end
						
						range_2_state <= 1;
					end
					else if(flag == 2 && range_2_state == 1)
					begin
						
						// traversing in a 16-wide parallel lane for the line fetched in flag = 1 state
						if(flag_1_or_2 == 1)
						begin
							
							if(no_cached_blocks_flag_1 > 0)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[0] == input_char)
									next[block_mem_state_info_target_state[0]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[0] == input_char_2)
									next_2[block_mem_state_info_target_state[0]] <= 1;
							end
							
							if(no_cached_blocks_flag_1 > 1)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[1] == input_char)
									next[block_mem_state_info_target_state[1]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[1] == input_char_2)
									next_2[block_mem_state_info_target_state[1]] <= 1;
							end
							
							if(no_cached_blocks_flag_1 > 2)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[2] == input_char)
									next[block_mem_state_info_target_state[2]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[2] == input_char_2)
									next_2[block_mem_state_info_target_state[2]] <= 1;
							end
							
							if(no_cached_blocks_flag_1 > 3)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[3] == input_char)
									next[block_mem_state_info_target_state[3]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[3] == input_char_2)
									next_2[block_mem_state_info_target_state[3]] <= 1;
							end
							
							/********* First one till 15*******/
							
						end
						// traversing in a 16-wide parallel lane for the line fetched in flag = 2 state
						if(flag_1_or_2 == 2)
						begin
							if(no_cached_blocks_flag_2 > 0)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[0] == input_char)
									next[block_mem_state_info_target_state[0]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[0] == input_char_2)
									next_2[block_mem_state_info_target_state[0]] <= 1;
							end
							
							if(no_cached_blocks_flag_2 > 1)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[1] == input_char)
									next[block_mem_state_info_target_state[1]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[1] == input_char_2)
									next_2[block_mem_state_info_target_state[1]] <= 1;
							end
							
							if(no_cached_blocks_flag_2 > 2)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[2] == input_char)
									next[block_mem_state_info_target_state[2]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[2] == input_char_2)
									next_2[block_mem_state_info_target_state[2]] <= 1;
							end
							
							if(no_cached_blocks_flag_2 > 3)
							begin
								if(current[i] == 1 && block_mem_state_info_transition[3] == input_char)
									next[block_mem_state_info_target_state[3]] <= 1;
									
								if(current_2[i] == 1 && block_mem_state_info_transition[3] == input_char_2)
									next_2[block_mem_state_info_target_state[3]] <= 1;
							end
							
							/********* First one till 15*******/
							
						end
						
						range_2_state <= 0;
						state <= 4;
						flag <= 0;
					end
					else
					begin
					//----------------------accepting state-------------------
						state <= 4;
						flag <= 0;
					end
					
				end
			
			end
			else if(state == 4)
			begin
				
				accepting_match_flag <= 0;
				accepting_match_flag_2 <= 0;
				flag_2 <= 0;
				state <= 0;
				
				if(i<size-1)
				begin
					//check for next state
					i <= i + 1;
				end				
				else
				begin
					//all states traversed, fetch a new character.
					current <= next;
					current_2 <= next_2;
			
					next <= 0;
					next_2 <= 0;
					
					i <= 0;
					input_char_flag <= 1;
				end
				
			end
			else if(current[i] != 1 && current_2[i] !=1 && state == 0)
			begin
				
				if(i<size-1)
				begin
					//check for next state
					input_char_flag <= 0;
					i <= i + 1;
				end				
				else
				begin
					//all states traversed, fetch a new character.
					current <= next;
					current_2 <= next_2;
					
					next <= 0;
					next_2 <= 0;
					
					i <= 0;
					input_char_flag <= 1;  
				end
			end
			
		end
	end


	always@(*)
	begin		
		offset =  size + 1;
		
		rd_address_int = 32'bz;
		block_offset = 2'bz;
		block_offset_plus_one = 5'bz;
		cache_line_no = 19'bz;
		
		if((current[i] == 1 || current_2[i] == 1) && state ==0)
		begin
			rd_address_int = i;
			block_offset = rd_address_int[1:0];
			block_offset_plus_one = block_offset + 1'b1;
			cache_line_no = rd_address_int >> 2;
		end
		
		if(flag == 0 && state == 3 && range > 0)
		begin
			
			//fetching next line and extarcting the useful blocks
			
			rd_address_int = offset + up_counter;
			block_offset = rd_address_int[1:0];
			cache_line_no = rd_address_int >> 2;
			
			//checking if the no of useful blocks >==< range
			no_cached_blocks_int = 4 - block_offset;
			
			if(range>no_cached_blocks_int)
				no_cached_blocks = no_cached_blocks_int;
			else
				no_cached_blocks = range;
				
			//updating the upcounter according to no of useful cached blocks
			if(range > no_cached_blocks)
				up_counter_int = up_counter + no_cached_blocks;
			else
				up_counter_int = up_counter + range;
				
			//updating the range according to no of useful cached blocks
			if(range > no_cached_blocks)
				range_int = range - no_cached_blocks;
			else
				range_int = range - range;
				
		end
		else if(flag == 1 && state == 3 && range > 0)
		begin
			//fetching next line and extarcting the useful blocks
			cache_line_no = rd_address + 1;
			
			//no_cached_blocks_int = 8 - block_offset;
			
			//checking if the no of useful blocks >==< range
			if(range > 4)
				no_cached_blocks = 4;
			else
				no_cached_blocks = range;
				
			//updating the upcounter according to no of useful cached blocks
			if(range > 4)
				up_counter_int = up_counter + 4;
			else
				up_counter_int = up_counter + range;
			
			//updating the range according to no of useful cached blocks
			if(range > 4)
				range_int = range - 4;
			else
				range_int = range - range;
				
		end
		else if(flag == 2 && state == 3 && range > 0)
		begin
			//fetching next line and extarcting the useful blocks
			cache_line_no = rd_address + 1;
			
			//checking if the no of useful blocks >==< range
			if(range > 4)
				no_cached_blocks = 4;
			else
				no_cached_blocks = range;
				
			//updating the upcounter according to no of useful cached blocks
			if(range > 4)
				up_counter_int = up_counter + 4;
			else
				up_counter_int = up_counter + range;
			
			//updating the range according to no of useful cached blocks
			if(range > 4)
				range_int = range - 4;
			else
				range_int = range - range;
				
		end
		else
		begin
			no_cached_blocks = 5'bz;
			up_counter_int = 24'bz;
			range_int = 24'bz;
		end
	end
	
	always@(*)
	begin
		
		//reading bus into cache
		// lot 1
		cache[3] = rd_bus[mem0:0];
		cache[2] = rd_bus[mem1:mem1_e];
		cache[1] = rd_bus[mem2:mem2_e];
		cache[0] = rd_bus[mem3:mem3_e];
		
		//transitions
		// lot 1
		block_mem_state_info_transition[0] = cache[0][31:24];
		block_mem_state_info_transition[1] = cache[1][31:24];
		block_mem_state_info_transition[2] = cache[2][31:24];
		block_mem_state_info_transition[3] = cache[3][31:24];
		
		// target states
		// lot 1
		block_mem_state_info_target_state[0] = cache[0][23:0];
		block_mem_state_info_target_state[1] = cache[1][23:0];
		block_mem_state_info_target_state[2] = cache[2][23:0];
		block_mem_state_info_target_state[3] = cache[3][23:0];
		
	end
	
endmodule
