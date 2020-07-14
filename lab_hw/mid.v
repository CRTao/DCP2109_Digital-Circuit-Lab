`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2017/10/13 15:45:35
// Design Name: 
// Module Name: mid
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module mid(
  input  clk,
  input  reset_n,
  input  [3:0] usr_btn,
  output [3:0] usr_led,
  input  uart_rx,
  output uart_tx,
  output LCD_RS,
  output LCD_RW,
  output LCD_E,
  output [3:0] LCD_D

    );

reg [3:0] test_sequence [15:0];
reg [7:0] test_sequence_ASC [15:0];
reg [7:0] LCDB [15:0];
wire btn_level_0, btn_pressed_0;
wire btn_level_1, btn_pressed_1;
reg prev_btn_level_0;
reg prev_btn_level_1;
reg [127:0] row_A, row_B;

// declare UART signals
wire transmit;
wire received;
wire [7:0] rx_byte;
reg  [7:0] rx_temp;
wire [7:0] tx_byte;
wire is_receiving;
wire is_transmitting;
wire recv_error;


//--------------Q1 para-------------
reg [3:0] counterQ1 = 0;
//-------------Q2 para--------------
reg [3:0] maxcounter = 0;
reg [3:0] idx0 =0 ,idx1 = 1,midx=0;
reg [3:0] checkicr = 0;
//-------------Q3 para-------------
localparam [1:0] S_MAIN_INIT = 0, S_MAIN_FIR = 1,
               S_MAIN_SEC = 2, S_MAIN_DONE = 3;
localparam [1:0] S_UART_IDLE = 0, S_UART_WAIT = 1,
               S_UART_SEND = 2, S_UART_INCR = 3;
reg [1:0] P, P_next;
reg [1:0] Q, Q_next;
wire print_enable, print_done;
reg [28:0] init_counter;
reg [7:0] data[0:23];
reg [4:0] send_counter = 0;
reg [7:0] FC,SC;
reg [7:0] out1,out2;
localparam FIR_STR = 0;
//--------------------- initial ---------------
initial begin
    LCDB[ 0]=8'd32;
    LCDB[ 1]=8'd32;
    LCDB[ 2]=8'd32;
    LCDB[ 3]=8'd32;
    LCDB[ 4]=8'd32;
    LCDB[ 5]=8'd32;
    LCDB[ 6]=8'd32;
    LCDB[ 7]=8'd32;
    LCDB[ 8]=8'd32;
    LCDB[ 9]=8'd32;
    LCDB[10]=8'd32;
    LCDB[11]=8'd32;
    LCDB[12]=8'd32;
    LCDB[13]=8'd32;
    LCDB[14]=8'd32;
    LCDB[15]=8'd32;
end
//-----------------------------------------------------------------
always @(*) begin
	test_sequence[ 0] = 4'hC;
	test_sequence[ 1] = 4'hB;
	test_sequence[ 2] = 4'hA;
	test_sequence[ 3] = 4'hB;
	test_sequence[ 4] = 4'h1;
	test_sequence[ 5] = 4'h3; //*
	test_sequence[ 6] = 4'h5;
	test_sequence[ 7] = 4'h2;
	test_sequence[ 8] = 4'h4;
	test_sequence[ 9] = 4'h6;
	test_sequence[10] = 4'h8;
	test_sequence[11] = 4'hB;	
	test_sequence[12] = 4'h0;
	test_sequence[13] = 4'h1; //*
	test_sequence[14] = 4'h2;
	test_sequence[15] = 4'h3;
end
//--------------------------------------------------------------------------
always @ (*) begin

	if( test_sequence[ 0] < 10 ) test_sequence_ASC[ 0] = {4'b0000,test_sequence[ 0]} + 8'h30 ;  else test_sequence_ASC[ 0] = {4'b0000,test_sequence[ 0]} + 8'h37 ;
	if( test_sequence[ 1] < 10 ) test_sequence_ASC[ 1] = {4'b0000,test_sequence[ 1]} + 8'h30 ;  else test_sequence_ASC[ 1] = {4'b0000,test_sequence[ 1]} + 8'h37 ;
    if( test_sequence[ 2] < 10 ) test_sequence_ASC[ 2] = {4'b0000,test_sequence[ 2]} + 8'h30 ;  else test_sequence_ASC[ 2] = {4'b0000,test_sequence[ 2]} + 8'h37 ;
    if( test_sequence[ 3] < 10 ) test_sequence_ASC[ 3] = {4'b0000,test_sequence[ 3]} + 8'h30 ;  else test_sequence_ASC[ 3] = {4'b0000,test_sequence[ 3]} + 8'h37 ;	
    if( test_sequence[ 4] < 10 ) test_sequence_ASC[ 4] = {4'b0000,test_sequence[ 4]} + 8'h30 ;  else test_sequence_ASC[ 4] = {4'b0000,test_sequence[ 4]} + 8'h37 ;
    if( test_sequence[ 5] < 10 ) test_sequence_ASC[ 5] = {4'b0000,test_sequence[ 5]} + 8'h30 ;  else test_sequence_ASC[ 5] = {4'b0000,test_sequence[ 5]} + 8'h37 ;
    if( test_sequence[ 6] < 10 ) test_sequence_ASC[ 6] = {4'b0000,test_sequence[ 6]} + 8'h30 ;  else test_sequence_ASC[ 6] = {4'b0000,test_sequence[ 6]} + 8'h37 ;
    if( test_sequence[ 7] < 10 ) test_sequence_ASC[ 7] = {4'b0000,test_sequence[ 7]} + 8'h30 ;  else test_sequence_ASC[ 7] = {4'b0000,test_sequence[ 7]} + 8'h37 ;    
    if( test_sequence[ 8] < 10 ) test_sequence_ASC[ 8] = {4'b0000,test_sequence[ 8]} + 8'h30 ;  else test_sequence_ASC[ 8] = {4'b0000,test_sequence[ 8]} + 8'h37 ;
    if( test_sequence[ 9] < 10 ) test_sequence_ASC[ 9] = {4'b0000,test_sequence[ 9]} + 8'h30 ;  else test_sequence_ASC[ 9] = {4'b0000,test_sequence[ 9]} + 8'h37 ;
    if( test_sequence[10] < 10 ) test_sequence_ASC[10] = {4'b0000,test_sequence[10]} + 8'h30 ;  else test_sequence_ASC[10] = {4'b0000,test_sequence[10]} + 8'h37 ;
    if( test_sequence[11] < 10 ) test_sequence_ASC[11] = {4'b0000,test_sequence[11]} + 8'h30 ;  else test_sequence_ASC[11] = {4'b0000,test_sequence[11]} + 8'h37 ;    
    if( test_sequence[12] < 10 ) test_sequence_ASC[12] = {4'b0000,test_sequence[12]} + 8'h30 ;  else test_sequence_ASC[12] = {4'b0000,test_sequence[12]} + 8'h37 ;
    if( test_sequence[13] < 10 ) test_sequence_ASC[13] = {4'b0000,test_sequence[13]} + 8'h30 ;  else test_sequence_ASC[13] = {4'b0000,test_sequence[13]} + 8'h37 ;
    if( test_sequence[14] < 10 ) test_sequence_ASC[14] = {4'b0000,test_sequence[14]} + 8'h30 ;  else test_sequence_ASC[14] = {4'b0000,test_sequence[14]} + 8'h37 ;
    if( test_sequence[15] < 10 ) test_sequence_ASC[15] = {4'b0000,test_sequence[15]} + 8'h30 ;  else test_sequence_ASC[15] = {4'b0000,test_sequence[15]} + 8'h37 ;

end
//  ---------------------------Q1-----------------------------------
assign usr_led = test_sequence[counterQ1];
always@ (posedge clk) begin
if (~reset_n)
    counterQ1 <= 4'd0;
else if (btn_pressed_0 && counterQ1 != 4'b1111)
    counterQ1 <= counterQ1 + 1;
else if (btn_pressed_1 && counterQ1 != 4'b0000)
    counterQ1 <= counterQ1 - 1;
end

// --------------------------Q2-------------------------------------
always @(posedge clk)begin
    
    row_A <= {test_sequence_ASC[ 0],test_sequence_ASC[ 1],test_sequence_ASC[ 2],test_sequence_ASC[ 3]
            ,test_sequence_ASC[ 4],test_sequence_ASC[ 5],test_sequence_ASC[ 6],test_sequence_ASC[ 7]
            ,test_sequence_ASC[ 8],test_sequence_ASC[ 9],test_sequence_ASC[10],test_sequence_ASC[11]
            ,test_sequence_ASC[12],test_sequence_ASC[13],test_sequence_ASC[14],test_sequence_ASC[15]};
    row_B <= {LCDB[ 0],LCDB[ 1],LCDB[ 2],LCDB[ 3]
            ,LCDB[ 4],LCDB[ 5],LCDB[ 6],LCDB[ 7]
            ,LCDB[ 8],LCDB[ 9],LCDB[10],LCDB[11]
            ,LCDB[12],LCDB[13],LCDB[14],LCDB[15]};

end
//================= Algorithm =====================
always @(posedge clk)begin
    if(idx0!=4'hF)begin
        if(test_sequence[idx1-1] > test_sequence[idx1] && idx1-idx0==1 )begin
            idx0=idx0+1;
            idx1=idx1+1;
        end
        else if( (test_sequence[idx1-1] > test_sequence[idx1] && idx1-idx0!=1)|| idx1 == 4'd15 )begin
            if(idx1-idx0>maxcounter)begin
            maxcounter= idx1-idx0-1;
            midx=idx0;
            end
            if(idx1 == 4'd15 && test_sequence[idx1-1] < test_sequence[idx1] && idx1-idx0 > maxcounter)begin
            maxcounter = maxcounter+1;
            end
            idx0=idx0+1;
            idx1=idx0+1;
            end
        else if(test_sequence[idx1-1] < test_sequence[idx1] && idx1 != 4'd15 )begin
            idx1=idx1+1;
        end
    end
    else begin
        LCDB[midx]=8'd42;
        LCDB[midx+maxcounter]=8'd42;
    end
end

// ---------------------------------------Q3-------------------------------------------------
always @(posedge clk) begin
  if (P == S_MAIN_INIT) init_counter <= init_counter + 1;
  else init_counter <= 0;
end

always @(posedge clk) begin
  if (~reset_n) P <= S_MAIN_INIT;
  else P <= P_next;
end

always @(posedge clk) begin
  if (~reset_n) Q <= S_UART_IDLE;
  else Q <= Q_next;
end

always @(*) begin
  case (P)
    S_MAIN_INIT:
     begin    
	   if (init_counter < 100000) P_next = S_MAIN_INIT;
	   else P_next = S_MAIN_FIR;
	 end
    S_MAIN_FIR:
      if (print_done) P_next = S_MAIN_DONE;
      else P_next = S_MAIN_FIR;
    S_MAIN_DONE:
        P_next = S_MAIN_DONE;
  endcase
end

always @(*) begin
  case (Q)
    S_UART_IDLE:
      if (print_enable) Q_next = S_UART_WAIT;
      else Q_next = S_UART_IDLE;
    S_UART_WAIT:
      if (is_transmitting == 1) Q_next = S_UART_SEND;
      else Q_next = S_UART_WAIT;
    S_UART_SEND:
      if (is_transmitting == 0) Q_next = S_UART_INCR;
      else Q_next = S_UART_SEND;
    S_UART_INCR:
      if (print_done) Q_next = S_UART_IDLE;
      else Q_next = S_UART_WAIT;
  endcase
end

always @(posedge clk) begin
  case (P_next)
    S_MAIN_INIT: send_counter <= 0;
    default: send_counter <= send_counter + (Q_next == S_UART_INCR);
  endcase
end

assign print_enable = (P != S_MAIN_FIR && P_next == S_MAIN_FIR) ;
assign print_done = (tx_byte == 8'h0);
assign transmit = (Q_next == S_UART_WAIT || print_enable);
assign tx_byte = data[send_counter];

always @(posedge clk)begin
    
    FC [7:0] = { 4'b0000,midx } ;
    SC [7:0] = { 4'b0000,midx+maxcounter };
    if( FC<10 ) out1 = FC + 8'h30 ;  else out1 = FC + 8'h37 ;
    if( SC<10 ) out2 = SC + 8'h30 ;  else out2 = SC + 8'h37 ;
    
    { data[ 0], data[ 1], data[ 2], data[ 3], data[ 4], data[ 5], data[ 6], data[ 7],
      data[ 8], data[ 9], data[10], data[11], data[12], data[13], data[14], data[15],
      data[16], data[17], data[18], data[19], data[20],data[21],data[22],data[23]}
      <= {test_sequence_ASC[ 0],test_sequence_ASC[ 1],test_sequence_ASC[ 2],test_sequence_ASC[ 3]
         ,test_sequence_ASC[ 4],test_sequence_ASC[ 5],test_sequence_ASC[ 6],test_sequence_ASC[ 7]
         ,test_sequence_ASC[ 8],test_sequence_ASC[ 9],test_sequence_ASC[10],test_sequence_ASC[11]
         ,test_sequence_ASC[12],test_sequence_ASC[13],test_sequence_ASC[14],test_sequence_ASC[15],8'h0D, 8'h0A,out1,8'd32,out2,8'h0D, 8'h0A,8'h00};         
end




//-------------------------------------------------------------------------------------------------------------------

  uart uart(
  .clk(clk),
  .rst(~reset_n),
  .rx(uart_rx),
  .tx(uart_tx),
  .transmit(transmit),
  .tx_byte(tx_byte),
  .received(received),
  .rx_byte(rx_byte),
  .is_receiving(is_receiving),
  .is_transmitting(is_transmitting),
  .recv_error(recv_error)
);

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
      .btn_output(btn_level_0)
    );    
    debounce btn_db1(
      .clk(clk),
      .btn_input(usr_btn[1]),
      .btn_output(btn_level_1)
    );
        
    always @(posedge clk) begin
      if (~reset_n)
        prev_btn_level_0 <= 1;
      else
        prev_btn_level_0 <= btn_level_0;
    end
    always @(posedge clk) begin
      if (~reset_n)
        prev_btn_level_1 <= 1;
      else
        prev_btn_level_1 <= btn_level_1;
    end
    
    assign btn_pressed_0 = (btn_level_0 == 1 && prev_btn_level_0 == 0)? 1 : 0;
    assign btn_pressed_1 = (btn_level_1 == 1 && prev_btn_level_1 == 0)? 1 : 0;	

endmodule
