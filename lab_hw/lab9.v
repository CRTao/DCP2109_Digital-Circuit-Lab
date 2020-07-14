`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of Computer Science, National Chiao Tung University
// Engineer: Chun-Jen Tsai
// 
// Create Date: 2017/12/06 20:44:08
// Design Name: 
// Module Name: lab9
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: This is a sample circuit to show you how to initialize an SRAM
//              with a pre-defined data file. Hit BTN0/BTN1 let you browse
//              through the data.
// 
// Dependencies: LCD_module, debounce
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module lab9(
  // General system I/O ports
  input  clk,
  input  reset_n,
  input  [3:0] usr_btn,
  output [3:0] usr_led,

  // 1602 LCD Module Interface
  output LCD_RS,
  output LCD_RW,
  output LCD_E,
  output [3:0] LCD_D
);

localparam [2:0] S_MAIN_INIT = 3'b000, S_MAIN_READ = 3'b001,
                 S_MAIN_ADDR = 3'b010, S_MAIN_NEXT = 3'b011,
                 S_MAIN_CALC = 3'b100, S_MAIN_SHOW = 3'b101;

// declare system variables
wire              btn_level, btn_pressed;
reg               prev_btn_level;
reg  [2:0]        P, P_next;
reg  [11:0]       sample_addr;

reg  [127:0]      row_A= "Press BTN0 to do";
reg  [127:0]      row_B= "x-correlation...";

// declare SRAM control signals
wire [10:0]       sram_addr;
wire [7:0]        data_in;
wire [7:0]        data_out;
wire              sram_we, sram_en;

//self reg
reg  signed [7:0] data[0:1087] ;
reg  [11:0]       x;
reg  [7:0]        k;
reg  signed [23:0]tmp;
reg  signed [23:0]sum;
reg  signed [23:0]max;
reg  [11: 0]      max_pos;
reg               done;
reg               init;

assign usr_led = 4'b0;

// ------------------------------------------------------------------------
LCD_module lcd0( 
  .clk(clk),
  .reset(~reset_n),
  .row_A(row_A),
  .row_B(row_B),
  .LCD_E(LCD_E),
  .LCD_RS(LCD_RS),
  .LCD_RW(LCD_RW),
  .LCD_D(LCD_D)
);
  
debounce btn_db0(
  .clk(clk),
  .btn_input(usr_btn[0]),
  .btn_output(btn_level)
);

// ------------------------------------------------------------------------
always @(posedge clk) begin
  if (~reset_n) prev_btn_level <= 1'b0;
  else  prev_btn_level <= btn_level;
end
assign btn_pressed = (btn_level & ~prev_btn_level);
// ------------------------------------------------------------------------
sram ram0(.clk(clk), .we(sram_we), .en(sram_en),
          .addr(sram_addr), .data_i(data_in), .data_o(data_out));

assign sram_we = usr_btn[3];
assign sram_en = (P == S_MAIN_ADDR || P == S_MAIN_READ);
assign sram_addr = sample_addr[11:0];
assign data_in = 8'b0; 
// ------------------------------------------------------------------------
always @(posedge clk) begin
  if (~reset_n || P == S_MAIN_INIT)
    sample_addr = 12'h000;
  else if ( P == S_MAIN_ADDR && P_next == S_MAIN_NEXT )
    sample_addr = sample_addr + 1 ;
end
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// FSM of the main controller
always @(posedge clk) begin
  if (~reset_n) P <= S_MAIN_INIT; // read samples at 000 first
  else P <= P_next;
end

always @(*) begin // FSM next-state logic
    case (P)
    S_MAIN_INIT:
        if(btn_pressed)  P_next = S_MAIN_READ;
        else  P_next = S_MAIN_INIT;
    S_MAIN_READ:    P_next = S_MAIN_ADDR;   
    S_MAIN_ADDR: 
        if(sample_addr<=1088) P_next = S_MAIN_NEXT;
        else  P_next = S_MAIN_CALC;
    S_MAIN_NEXT:    P_next = S_MAIN_READ;       
    S_MAIN_CALC:
        if(done)    P_next = S_MAIN_SHOW;
        else    P_next = S_MAIN_CALC;
    S_MAIN_SHOW:    P_next = S_MAIN_SHOW;
    endcase
end

always @(posedge clk) begin // FSM next-state logic
    if(P==S_MAIN_INIT) begin
        x=0;k=0;init=0;done=0;
        sum=0;tmp=0;
        max=0;max_pos=0;		
    end
    else if(P==S_MAIN_READ) begin
        if(sram_en&&!sram_we) data[sample_addr] <= data_out;
    end
    else if(P==S_MAIN_CALC) begin
        if(k<64) begin
            if(~init) begin
                 init=1;
                 tmp=data[x+k]*data[1024+k];
            end
            else begin 
                 init=0;	
                 sum=sum+tmp;
                 k=k+1;
            end
            /*
            sum=sum+data[x+k]*data[1024+k];
            k=k+1;            
            */            
        end
        else    begin
            if(sum>max) begin
                 max=sum;
                 max_pos=x;
            end
            k=0;
            sum=0;
            x=x+1;
            if(x>=1024-64) begin
                 done=1;
            end
        end
    end
end

always @(posedge clk) begin // FSM next-state logic
    if(P==S_MAIN_INIT)  begin
        row_A <= "Press BTN0 to do";
        row_B <= "x-correlation...";          
    end    
    else if(P==S_MAIN_SHOW) begin
        row_A <= {"Max value ",
              ((max[23:20]>9)?"7":"0")+max[23:20],
              ((max[19:16]>9)?"7":"0")+max[19:16],
              ((max[15:12]>9)?"7":"0")+max[15:12],
              ((max[11:08]>9)?"7":"0")+max[11:08],
              ((max[07:04]>9)?"7":"0")+max[07:04],
              ((max[03:00]>9)?"7":"0")+max[03:00]};
        row_B <= {"Max location ",
              ((max_pos[11:08]>9)?"7":"0")+max_pos[11:08],
              ((max_pos[07:04]>9)?"7":"0")+max_pos[07:04],
              ((max_pos[03:00]>9)?"7":"0")+max_pos[03:00]};     
    end
end

// End of the main controller
// ------------------------------------------------------------------------

endmodule
