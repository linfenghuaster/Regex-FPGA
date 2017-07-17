//Copyright 1986-2016 Xilinx, Inc. All Rights Reserved.
//--------------------------------------------------------------------------------
//Tool Version: Vivado v.2016.2 (lin64) Build 1577090 Thu Jun  2 16:32:35 MDT 2016
//Date        : Sun Jul 16 12:35:56 2017
//Host        : nps2.ece.ncsu.edu running 64-bit Red Hat Enterprise Linux Workstation release 7.3 (Maipo)
//Command     : generate_target design_1_wrapper.bd
//Design      : design_1_wrapper
//Purpose     : IP block netlist
//--------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

module design_1_wrapper
   (BRAM_PORTA_addr,
    BRAM_PORTA_clk,
    BRAM_PORTA_dout);
  input [4:0]BRAM_PORTA_addr;
  input BRAM_PORTA_clk;
  output [511:0]BRAM_PORTA_dout;

  wire [4:0]BRAM_PORTA_addr;
  wire BRAM_PORTA_clk;
  wire [511:0]BRAM_PORTA_dout;

  design_1 design_1_i
       (.BRAM_PORTA_addr(BRAM_PORTA_addr),
        .BRAM_PORTA_clk(BRAM_PORTA_clk),
        .BRAM_PORTA_dout(BRAM_PORTA_dout));
endmodule
