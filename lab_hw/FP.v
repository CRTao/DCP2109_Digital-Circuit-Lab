`timescale 1ns / 1ps
module FP(
    input  clk,
    input  reset_n,
    input  [3:0] usr_btn,
    output [3:0] usr_led,

    output LCD_RS,
    output LCD_RW,
    output LCD_E,
    output [3:0] LCD_D
    );


localparam [3:0] S_MAIN_INIT = 0, S_MAIN_WAIT_KEY = 1,
                 S_MAIN_Q1 = 2 , S_MAIN_Q1_O = 3,
                 S_MAIN_Q2 = 4 , S_MAIN_Q3 = 5,
                 S_MAIN_WAIT_K = 6, S_MAIN_Q2_O = 7;
                 
reg [3:0] P, P_next;
reg [23:0] init_counter;

// declare SRAM control signals
wire [2:0] data_in;
wire sram_we, sram_en; 
wire [11:0] data_out;

//button signal
wire btn_level, btn_pressed;
reg prev_btn_level;

//LCD signal
reg [127:0] row_A; 
reg [127:0] row_B; 

//image1 sram signal 320*240
reg [16:0] image1_addr;
wire [2:0] image1_data_out;

//image2 sram signal 10*10
reg [6:0]image2_addr;
wire [2:0]image2_data_out;

// Declare the video buffer size
localparam VBUF_W = 320; //image1 width
localparam VBUF_H = 240; //image1 height
localparam PIC_W = 10;   //image2 width
localparam PIC_H = 10;   //image2 height

// ------------------------------------------------------------------------
// The following code describes an initialized SRAM memory block that
// stores an 320x240 3-bit  image

assign sram_en = 1'b1;
assign data_in = 3'd0;
assign sram_we = usr_btn[1];
assign usr_led = 4'd0;


sram #(.DATA_WIDTH(3),
	   .ADDR_WIDTH(17),
	   .RAM_SIZE(VBUF_W*VBUF_H),
	   .INDEX(0)
	   )
  ram0 (
		.clk(clk),
        .we(sram_we),
		.en(sram_en),
        .addr(image1_addr),
		.data_i(data_in),
		.data_o(image1_data_out)
		);
          
sram #(.DATA_WIDTH(3),
	   .ADDR_WIDTH(7),
	   .RAM_SIZE(PIC_W*PIC_H),
       .INDEX(1)
	   )
  ram1 (
		.clk(clk),
        .we(sram_we),
		.en(sram_en),
        .addr(image2_addr),
		.data_i(data_in),
		.data_o(image2_data_out)
		);       


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
    
always @(posedge clk) begin
  if (~reset_n)
    prev_btn_level <= 1;
  else
    prev_btn_level <= btn_level;
end

assign btn_pressed = (btn_level == 1 && prev_btn_level == 0);

//===================================

reg  [11:0] Q1_reg;  // RGB value for the current pixel
reg  [11:0] Q1_next; // RGB value for the next pixel
reg  cnt_Q1;
reg  [11:0] Q2_next;
reg  [11:0] Q2_reg;
reg  [7:0] cntcnt_Q1 = 0;
reg  [19:0] cntcnt_Q2 = 0;
reg  [11:0] Q1_a = 0;
reg  [1:0]  ls=0,lt=0;
reg  [11:0] x1,y1;
reg  [11:0] x2,y2;




always @(posedge clk) begin
    if(P==S_MAIN_INIT)begin
        row_A <= "DLAB Final      ";
        row_B <= "Press button 0  ";
    end
    else if( P == S_MAIN_Q1_O )begin
        row_A = {"The sum is ",
                (Q1_a[11:8]>9 ? 8'h37 : 8'h30 ) + Q1_a[11:8],
                (Q1_a[ 7:4]>9 ? 8'h37 : 8'h30 ) + Q1_a[ 7:4],
                (Q1_a[ 3:0]>9 ? 8'h37 : 8'h30 ) + Q1_a[ 3:0],". "};
        row_B = "Press button 0  ";
    end
    else if( P == S_MAIN_Q2_O )begin
        row_A = {"Start:(",
                    (x1[11:8]>9 ? 8'h37 : 8'h30 ) + x1[11:8],
                    (x1[ 7:4]>9 ? 8'h37 : 8'h30 ) + x1[ 7:4],
                    (x1[ 3:0]>9 ? 8'h37 : 8'h30 ) + x1[ 3:0],
                    ",",
                    (y1[11:8]>9 ? 8'h37 : 8'h30 ) + y1[11:8],
                    (y1[ 7:4]>9 ? 8'h37 : 8'h30 ) + y1[ 7:4],
                    (y1[ 3:0]>9 ? 8'h37 : 8'h30 ) + y1[ 3:0],
                    ")."};
        row_B = {"  End:(",
                    (x2[11:8]>9 ? 8'h37 : 8'h30 ) + x2[11:8],
                    (x2[ 7:4]>9 ? 8'h37 : 8'h30 ) + x2[ 7:4],
                    (x2[ 3:0]>9 ? 8'h37 : 8'h30 ) + x2[ 3:0],
                    ",",
                    (y2[11:8]>9 ? 8'h37 : 8'h30 ) + y2[11:8],
                    (y2[ 7:4]>9 ? 8'h37 : 8'h30 ) + y2[ 7:4],
                    (y2[ 3:0]>9 ? 8'h37 : 8'h30 ) + y2[ 3:0],
                    ")."};
    end
end

always @(posedge clk) begin
    if(~reset_n)begin
        Q1_a = 0;
        cntcnt_Q1 = 0;
    end
    else if( P == S_MAIN_Q1)begin
        Q1_reg = Q1_next;
        if( cntcnt_Q1!=0 )Q1_a = Q1_a + Q1_reg;
        cntcnt_Q1 <= cntcnt_Q1 + 1 ;
    end 
end

always @(posedge clk) begin
    if(~reset_n)begin
        cntcnt_Q2 = 0;
        ls=2'b00;
        lt=2'b00;
    end
    else if( P == S_MAIN_Q2)begin
        Q2_reg = Q2_next;
        
            if( Q2_reg == 4 && ls ==2'b00) ls=2'b01;  
            if( Q2_reg == 0 && ls ==2'b01)begin
                ls = 2'b11;
                //x1 = image1_addr / 240 + 1;
                //y1 = image1_addr - 240*(x1-1) -1 ; 
            end
            
            
            if( Q2_reg == 0 && lt ==2'b00) lt=2'b01;          
            if( Q2_reg == 4 && lt ==2'b01) begin
                lt = 2'b11;
                //x2 = image1_addr / 240 + 1;
                //y2 = image1_addr - 240*(x2-1) -1 ; 
            end
        
        cntcnt_Q2 <= cntcnt_Q2 + 1 ;
    end 
end

always @(*) begin
  if (~reset_n)
    Q1_next <= 12'h000; 
  else if( P == S_MAIN_Q1 )begin
    Q1_next <= image2_data_out;
  end
end
/*
always @(*) begin
  if (~reset_n)
    Q2_next <= 12'h000; 
  else if( P == S_MAIN_Q2 )begin
    Q2_next <= image1_data_out;
  end
end

always @(posedge clk) begin 
  if (~reset_n || P == S_MAIN_INIT ) image2_addr <= 8'h0;
  else if ( P == S_MAIN_Q1 ) 
  	image2_addr = image2_addr + 1;
end

always @(posedge clk) begin 
  if (~reset_n || P == S_MAIN_INIT ) image1_addr <= 17'h0;
  else if ( P == S_MAIN_Q2 ) 
  	image1_addr = image1_addr + 1;
end
*/
//===================================

always @(posedge clk) begin
  if (P == S_MAIN_INIT) init_counter <= init_counter + 1;
  else init_counter <= 0;
end

always @(posedge clk) begin
  if (~reset_n) P <= S_MAIN_INIT;
  else P <= P_next;
end

always @(*) begin // FSM next-state logic
  case (P)
    S_MAIN_INIT: // Delay 10 us.
     begin    
	   if (init_counter < 1000) P_next = S_MAIN_INIT;
	   else P_next = S_MAIN_Q1;
	 end
	S_MAIN_Q1:
       if ( cntcnt_Q1 >= 100 ) P_next = S_MAIN_WAIT_KEY;
       else P_next = S_MAIN_Q1;
    S_MAIN_WAIT_KEY: // wait for < Enter or number > key.
      if (btn_pressed) P_next = S_MAIN_Q1_O;
      else P_next = S_MAIN_WAIT_KEY;
    S_MAIN_Q1_O:
      if (btn_pressed) P_next = S_MAIN_Q2;
      else P_next = S_MAIN_Q1_O;
    S_MAIN_Q2:
      if ( cntcnt_Q2 >= 76800 ) P_next = S_MAIN_WAIT_K;
      else P_next = S_MAIN_Q2;
    S_MAIN_WAIT_K:
      if (btn_pressed) P_next = S_MAIN_Q2_O;
      else P_next = S_MAIN_WAIT_K;
    S_MAIN_Q2_O:
      if (btn_pressed) P_next = S_MAIN_Q3;
      else P_next = S_MAIN_Q2_O;
    S_MAIN_Q3:
        P_next = S_MAIN_Q3;
  endcase
end













endmodule
