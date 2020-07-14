`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of Computer Science, National Chiao Tung University
// Engineer: Chun-Jen Tsai
// 
// Create Date: 2017/05/08 15:29:41
// Design Name: 
// Module Name: lab6
// Project Name: 
// Target Devices: 
// Tool Versions:
// Description: The sample top module of lab 6: sd card reader. The behavior of
//              this module is as follows
//              1. When the SD card is initialized, display a message on the LCD.
//                 If the initialization fails, an error message will be shown.
//              2. The user can then press usr_btn[2] to trigger the sd card
//                 controller to read the super block of the sd card (located at
//                 block # 8192) into the SRAM memory.
//              3. During SD card reading time, the four LED lights will be turned on.
//                 They will be turned off when the reading is done.
//              4. The LCD will then displayer the sector just been read, and the
//                 first byte of the sector.
//              5. Everytime you press usr_btn[2], the next byte will be displayed.
// 
// Dependencies: clk_divider, LCD_module, debounce, sd_card
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module lab6(
  // General system I/O ports
  input  clk,
  input  reset_n,
  input  [3:0] usr_btn,
  output [3:0] usr_led,
  // SD card specific I/O ports
  output spi_ss,
  output spi_sck,
  output spi_mosi,
  input  spi_miso,
  // 1602 LCD Module Interface
  output LCD_RS,
  output LCD_RW,
  output LCD_E,
  output [3:0] LCD_D, 
  // Uart I/O
  input uart_rx,
  output uart_tx
);

localparam [3:0] S_MAIN_INIT = 4'b0000, S_BTN1_WAIT = 4'b0001,
                 S_MAIN_IDLE = 4'b0010, S_MAIN_WAIT = 4'b0011, 
                 S_MAIN_READ = 4'b0100, S_READ_DONE = 4'b0101,
                 S_FIND_MTAG = 4'b0110, S_MAIN_CALC  = 4'b0111,
				 S_ASCI_TRAS = 4'b1000, S_UART_PRIN  = 4'b1001,
				 S_MAIN_DONE  = 4'b1010;
				 
localparam [1:0] S_UART_IDLE = 2'b00, S_UART_WAIT = 2'b01, 
				 S_UART_SEND = 2'b10, S_UART_INCR = 2'b11;

// Declare system variables -------------------------------------ALL from old file
wire btn_level, btn_pressed;
reg  prev_btn_level;
reg  [5:0] send_counter;
reg  [3:0] P, P_next;
reg  [1:0] Q, Q_next;
reg  [9:0] uart_cnt = 0;
reg  [9:0] sd_counter;
reg  [7:0] data_byte;
reg  [31:0] blk_addr;
// Declare SD card interface signals
wire clk_sel;
wire clk_500k;
reg  rd_req;
reg  [31:0] rd_addr;
wire init_finished;
wire [7:0] sd_dout;
wire sd_valid;
// Declare the control/data signals of an SRAM memory block
wire [7:0] data_in;
wire [7:0] data_out;
wire [8:0] sram_addr;
wire       sram_we, sram_en;
// declare UART signals
wire transmit;
wire received;
wire [7:0] rx_byte;
wire [7:0] tx_byte;
wire is_receiving;
wire is_transmitting;
wire recv_error;
// LCD
reg  [127:0] row_A = "SD card cannot  ";
reg  [127:0] row_B = "be initialized! ";
reg  done_flag; // Signals the completion of reading one SD sector.
// new reg
reg [5:0] counter;
reg switch;
reg [16*8-1:0] A_mat;
reg [16*8-1:0] B_mat;
reg [16*20-1:0] C_mat;
reg [17:0] mu01, mu02, mu03, mu04, mu05, mu06, mu07, mu08, mu09, mu10, mu11, mu12, mu13, mu14, mu15, mu16;
reg [19:0] add1, add2, add3, add4;
reg calc_done, data_done;
integer i,j,k;

reg cnt_trigger;
reg MATIN_done;
reg [3:0] tag_idx;
reg all_done;
reg [7:0] data [0:147];
wire print_message;

assign clk_sel = (init_finished)? clk : clk_500k; // clock for the SD controller
assign usr_led = P;
assign print_message = (P == S_UART_PRIN) ? 1 : 0; // flag that triggers the operation of the UART controller

initial begin
	{ data[  0], data[  1], data[  2], data[  3], data[  4], data[  5], data[  6], data[  7],
	  data[  8], data[  9], data[ 10], data[ 11], data[ 12], data[ 13], data[ 14], data[ 15], 
	  data[ 16]}
	<= { "The result is: ", 8'h0D, 8'h0A};
	
	{ data[ 17], data[ 18], data[ 19], data[ 20], data[ 21], data[ 22], data[ 23], data[ 24], 
	  data[ 25], data[ 26], data[ 27], data[ 28], data[ 29], data[ 30], data[ 31], data[ 32], 
	  data[ 33], data[ 34], data[ 35], data[ 36], data[ 37], data[ 38], data[ 39], data[ 40],
	  data[ 41], data[ 42], data[ 43], data[ 44], data[ 45], data[ 46], data[ 47], data[ 48] }
	<= {"[ XXXXX, XXXXX, XXXXX, XXXXX ]", 8'h0D, 8'h0A};
	
	{ data[ 49], data[ 50], data[ 51], data[ 52], data[ 53], data[ 54], data[ 55], data[ 56], 
	  data[ 57], data[ 58], data[ 59], data[ 60], data[ 61], data[ 62], data[ 63], data[ 64],
	  data[ 65], data[ 66], data[ 67], data[ 68], data[ 69], data[ 70], data[ 71], data[ 72],
	  data[ 73], data[ 74], data[ 75], data[ 76], data[ 77], data[ 78], data[ 79], data[ 80] }
	<= {"[ XXXXX, XXXXX, XXXXX, XXXXX ]", 8'h0D, 8'h0A};
	
	{ data[ 81], data[ 82], data[ 83], data[ 84], data[ 85], data[ 86], data[ 87], data[ 88], 
	  data[ 89], data[ 90], data[ 91], data[ 92], data[ 93], data[ 94], data[ 95], data[ 96],
	  data[ 97], data[ 98], data[ 99], data[100], data[101], data[102], data[103], data[104],
	  data[105], data[106], data[107], data[108], data[109], data[110], data[111], data[112] }
	<= {"[ XXXXX, XXXXX, XXXXX, XXXXX ]", 8'h0D, 8'h0A};
	
	{ data[113], data[114], data[115], data[116], data[117], data[118], data[119], data[120],
	  data[121], data[122], data[123], data[124], data[125], data[126], data[127], data[128],
	  data[129], data[130], data[131], data[132], data[133], data[134], data[135], data[136],
	  data[137], data[138], data[139], data[140], data[141], data[142], data[143], data[144] }
	<= {"[ XXXXX, XXXXX, XXXXX, XXXXX ]", 8'h0D, 8'h0A};
	
	{ data[145], data[146], data[147] }
	<= {8'h0D, 8'h0A, 8'h00};
end

//---------------------------- module input ------------------------
clk_divider#(200) clk_divider0(
  .clk(clk),
  .reset(~reset_n),
  .clk_out(clk_500k)
);
debounce btn_db0(
  .clk(clk),
  .btn_input(usr_btn[1]),
  .btn_output(btn_level)
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
sd_card sd_card0(
  .cs(spi_ss),
  .sclk(spi_sck),
  .mosi(spi_mosi),
  .miso(spi_miso),
  .clk(clk_sel),
  .rst(~reset_n),
  .rd_req(rd_req),
  .block_addr(rd_addr),
  .init_finished(init_finished),
  .dout(sd_dout),
  .sd_valid(sd_valid)
);
sram ram0(
  .clk(clk),
  .we(sram_we),
  .en(sram_en),
  .addr(sram_addr),
  .data_i(data_in),
  .data_o(data_out)
);
uart uart0(
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
    
// ============== FSM =======================
always @(*) begin // FSM next-state logic
  case (P) 
    S_MAIN_INIT: // SD card initialization
      if (init_finished == 1) P_next = S_BTN1_WAIT;
      else P_next = S_MAIN_INIT;
    S_BTN1_WAIT: // wait for button click
      if (btn_pressed == 1) P_next = S_MAIN_IDLE;
      else P_next = S_BTN1_WAIT;
    S_MAIN_IDLE:  // check for matrix input
      if (!MATIN_done) P_next = S_MAIN_WAIT;
      else P_next = S_MAIN_CALC;
    S_MAIN_WAIT: // issue a rd_req to SD
      P_next = S_MAIN_READ;
    S_MAIN_READ: // wait for SRAM buffer 
      if (sd_counter == 512) P_next = S_READ_DONE;
      else P_next = S_MAIN_READ;
    S_READ_DONE: // read from sram
      if (!MATIN_done) P_next = S_FIND_MTAG;
      else P_next = S_MAIN_CALC;
    S_FIND_MTAG: // find MATX_TAG
      if (sd_counter < 512) P_next = S_READ_DONE;
      else P_next = S_MAIN_IDLE;
 	S_MAIN_CALC:  // calculate matrix
	  if (calc_done) P_next = S_ASCI_TRAS;
	  else P_next = S_MAIN_CALC;
	S_ASCI_TRAS:  //  put data into output buffer
	  if (data_done) P_next = S_UART_PRIN;
	  else P_next = S_ASCI_TRAS;
	S_UART_PRIN:  // uart print
      if (all_done) P_next = S_MAIN_DONE;
	  else P_next = S_UART_PRIN;
	S_MAIN_DONE:  // done
	  P_next = S_MAIN_DONE;
    default:
      P_next = S_MAIN_DONE;
  endcase
end
// ---------------------------SD Card Trigger--------------------------------------
always @(posedge clk) begin
  if (~reset_n) begin
    P <= S_MAIN_INIT;
    done_flag <= 0;
  end
  else begin
    P <= P_next;
    if (P == S_READ_DONE)
      done_flag <= 1;
    else if (P == S_FIND_MTAG && P_next == S_MAIN_IDLE)
      done_flag <= 0;
    else
      done_flag <= done_flag;
  end
end
// ------------------------ SD input --------------------------
always @(*) begin
    rd_req = (P == S_MAIN_WAIT);
    rd_addr = blk_addr;
end
// ------------------------- SD address ----------------------
always @(posedge clk) begin
    if (~reset_n) blk_addr <= 32'h2000;
    else if ((P == S_FIND_MTAG && P_next == S_MAIN_IDLE) && !MATIN_done) begin 
        blk_addr <= blk_addr + 1; // In lab 6, change this line to scan all blocks
    end
end
// ------------------------  SD byte counter ----------------------
always @(posedge clk) begin
  if (~reset_n || (P == S_MAIN_READ && P_next == S_READ_DONE) || (P == S_MAIN_WAIT && P_next == S_MAIN_READ))
    sd_counter <= 0;
  else if ((P == S_MAIN_READ && sd_valid) ||
           (P == S_READ_DONE && P_next == S_FIND_MTAG))
    sd_counter <= sd_counter + 1;
end
// ----------------------button debounce---------------------
always @(posedge clk) begin
  if (~reset_n)
    prev_btn_level <= 0;
  else
    prev_btn_level <= btn_level;
end
assign btn_pressed = (btn_level == 1 && prev_btn_level == 0)? 1 : 0;
// -------------------------- SRAM reg ----------------------------
assign sram_we = sd_valid;          // Write data into SRAM when sd_valid is high.
assign sram_en = 1;                 // Always enable the SRAM block.
assign data_in = sd_dout;           // Input data always comes from the SD controller.
assign sram_addr = sd_counter[8:0]; // Set the driver of the SRAM address signal.
// ------------------------ SRAM output ---------------------------
always @(posedge clk) begin
  if (~reset_n) data_byte <= 8'b0;
  else if (sram_en && P == S_READ_DONE) data_byte <= data_out;
end
//  ----------------------- Matrix Calculation ---------------------
always @(posedge clk) begin
    if (~reset_n || P == S_MAIN_INIT) begin
		counter = 0;
        cnt_trigger = 0;
        tag_idx = 0;
        all_done = 0;
        MATIN_done = 0;
		data_done = 0;
		switch = 0;
		calc_done = 0;
		i=127; j=0; k=0;
		mu01=0; mu02=0; mu03=0; mu04=0; mu05=0; mu06=0; mu07=0; mu08=0;
		mu09=0; mu10=0; mu11=0; mu12=0; mu13=0; mu14=0; mu15=0; mu16=0;
		add1=0; add2=0; add3=0; add4=0;
		A_mat=0; B_mat=0; C_mat=0;
	end else if (P == S_READ_DONE && P_next == S_FIND_MTAG) begin
        if      (tag_idx == 4'd0) begin if (data_byte == "M") tag_idx = tag_idx + 1;else tag_idx = 0; end
        else if (tag_idx == 4'd1) begin if (data_byte == "A") tag_idx = tag_idx + 1;else tag_idx = 0; end
        else if (tag_idx == 4'd2) begin if (data_byte == "T") tag_idx = tag_idx + 1;else tag_idx = 0; end
        else if (tag_idx == 4'd3) begin if (data_byte == "X") tag_idx = tag_idx + 1;else tag_idx = 0; end
        else if (tag_idx == 4'd4) begin if (data_byte == "_") tag_idx = tag_idx + 1;else tag_idx = 0; end
        else if (tag_idx == 4'd5) begin if (data_byte == "T") tag_idx = tag_idx + 1;else tag_idx = 0; end
        else if (tag_idx == 4'd6) begin if (data_byte == "A") tag_idx = tag_idx + 1;else tag_idx = 0; end
        else if (tag_idx == 4'd7) begin if (data_byte == "G") begin tag_idx = tag_idx + 1; cnt_trigger = 1; end
             else tag_idx = 0; end 
        else if (tag_idx == 4'd8 && cnt_trigger == 1) begin
            if (data_byte == 8'h0D || ((data_byte < 65 || data_byte > 70) && (data_byte < 48 || data_byte > 57))) counter = counter;
            else if (data_byte == 8'h0A || ((data_byte < 65 || data_byte > 70) && (data_byte < 48 || data_byte > 57))) counter = counter;
			else begin 
				if (switch == 0) begin // A
					if      (data_byte >= 65 && data_byte <= 70) A_mat[i-:4] = data_byte[3:0]-55;
					else if (data_byte >= 48 && data_byte <= 57) A_mat[i-:4] = data_byte[3:0]-48;
					if (i > 3) i = i - 4;
					counter = counter + 1;
					if (counter == 6'd32) begin
					   counter = 0;
					   switch = 1;
					   i = 127;
					end
				end 
				else if (switch == 1) begin // B
					if      (data_byte >= 65 && data_byte <= 70) B_mat[i-:4] = data_byte[3:0]-55;
					else if (data_byte >= 48 && data_byte <= 57) B_mat[i-:4] = data_byte[3:0]-48;
					if (i > 3) i = i - 4;
					counter = counter + 1;
					if (counter == 6'd32) begin
					   MATIN_done = 1;
					   cnt_trigger = 0;
					end
				end	
			end
        end
    end else if (P == S_MAIN_CALC && calc_done == 0) begin
        mu01 = A_mat[127-(4*0+j)*8 -: 8] * B_mat[127:120];
		mu02 = A_mat[127-(4*1+j)*8 -: 8] * B_mat[119:112];
		mu03 = A_mat[127-(4*2+j)*8 -: 8] * B_mat[111:104];
		mu04 = A_mat[127-(4*3+j)*8 -: 8] * B_mat[103: 96];
		mu05 = A_mat[127-(4*0+j)*8 -: 8] * B_mat[ 95: 88];
		mu06 = A_mat[127-(4*1+j)*8 -: 8] * B_mat[ 87: 80];
		mu07 = A_mat[127-(4*2+j)*8 -: 8] * B_mat[ 79: 72];
		mu08 = A_mat[127-(4*3+j)*8 -: 8] * B_mat[ 71: 64];
		mu09 = A_mat[127-(4*0+j)*8 -: 8] * B_mat[ 63: 56];
		mu10 = A_mat[127-(4*1+j)*8 -: 8] * B_mat[ 55: 48];
		mu11 = A_mat[127-(4*2+j)*8 -: 8] * B_mat[ 47: 40];
		mu12 = A_mat[127-(4*3+j)*8 -: 8] * B_mat[ 39: 32];
		mu13 = A_mat[127-(4*0+j)*8 -: 8] * B_mat[ 31: 24];
		mu14 = A_mat[127-(4*1+j)*8 -: 8] * B_mat[ 23: 16];
		mu15 = A_mat[127-(4*2+j)*8 -: 8] * B_mat[ 15:  8];
		mu16 = A_mat[127-(4*3+j)*8 -: 8] * B_mat[  7:  0];
		add1 = mu01 + mu02 + mu03 + mu04;
		add2 = mu05 + mu06 + mu07 + mu08;
		add3 = mu09 + mu10 + mu11 + mu12;
		add4 = mu13 + mu14 + mu15 + mu16;
		C_mat[319-(4*0+j)*20 -: 20] = add1;
		C_mat[319-(4*1+j)*20 -: 20] = add2;
		C_mat[319-(4*2+j)*20 -: 20] = add3;
		C_mat[319-(4*3+j)*20 -: 20] = add4;
		j = j + 1;
		if(j==4)calc_done = 1;
		else calc_done = 0;
    end else if (P == S_ASCI_TRAS) begin
		data[ 19+k] = (C_mat[319-(4*k) -: 4] > 9) ? C_mat[319-(4*k) -: 4] + 55 : C_mat[319-(4*k) -: 4] + 48;
		data[24] = ","; data[25] <= " ";
		data[ 26+k] = (C_mat[239-(4*k) -: 4] > 9) ? C_mat[239-(4*k) -: 4] + 55 : C_mat[239-(4*k) -: 4] + 48;
		data[31] = ","; data[32] <= " ";
		data[ 33+k] = (C_mat[159-(4*k) -: 4] > 9) ? C_mat[159-(4*k) -: 4] + 55 : C_mat[159-(4*k) -: 4] + 48;
		data[38] = ","; data[39] <= " ";
		data[ 40+k] = (C_mat[ 79-(4*k) -: 4] > 9) ? C_mat[ 79-(4*k) -: 4] + 55 : C_mat[ 79-(4*k) -: 4] + 48;
		data[45] = " ";	
		data[ 51+k] = (C_mat[299-(4*k) -: 4] > 9) ? C_mat[299-(4*k) -: 4] + 55 : C_mat[299-(4*k) -: 4] + 48;
		data[56] = ","; data[57] <= " ";
		data[ 58+k] = (C_mat[219-(4*k) -: 4] > 9) ? C_mat[219-(4*k) -: 4] + 55 : C_mat[219-(4*k) -: 4] + 48;
		data[63] = ","; data[64] <= " ";
		data[ 65+k] = (C_mat[139-(4*k) -: 4] > 9) ? C_mat[139-(4*k) -: 4] + 55 : C_mat[139-(4*k) -: 4] + 48;
		data[70] = ","; data[71] <= " ";
		data[ 72+k] = (C_mat[ 59-(4*k) -: 4] > 9) ? C_mat[ 59-(4*k) -: 4] + 55 : C_mat[ 59-(4*k) -: 4] + 48;
		data[77] = " ";	
		data[ 83+k] = (C_mat[279-(4*k) -: 4] > 9) ? C_mat[279-(4*k) -: 4] + 55 : C_mat[279-(4*k) -: 4] + 48;
		data[88] = ","; data[89] <= " ";
		data[ 90+k] = (C_mat[199-(4*k) -: 4] > 9) ? C_mat[199-(4*k) -: 4] + 55 : C_mat[199-(4*k) -: 4] + 48;
		data[95] = ","; data[96] <= " ";
		data[ 97+k] = (C_mat[119-(4*k) -: 4] > 9) ? C_mat[119-(4*k) -: 4] + 55 : C_mat[119-(4*k) -: 4] + 48;
		data[102] = ","; data[103] <= " ";
		data[104+k] = (C_mat[ 39-(4*k) -: 4] > 9) ? C_mat[ 39-(4*k) -: 4] + 55 : C_mat[ 39-(4*k) -: 4] + 48;
		data[109] = " ";		
		data[115+k] = (C_mat[259-(4*k) -: 4] > 9) ? C_mat[259-(4*k) -: 4] + 55 : C_mat[259-(4*k) -: 4] + 48;
		data[120] = ","; data[121] <= " ";
		data[122+k] = (C_mat[179-(4*k) -: 4] > 9) ? C_mat[179-(4*k) -: 4] + 55 : C_mat[179-(4*k) -: 4] + 48;
		data[127] = ","; data[128] <= " ";
		data[129+k] = (C_mat[ 99-(4*k) -: 4] > 9) ? C_mat[ 99-(4*k) -: 4] + 55 : C_mat[ 99-(4*k) -: 4] + 48;
		data[134] = ","; data[135] <= " ";
		data[136+k] = (C_mat[ 19-(4*k) -: 4] > 9) ? C_mat[ 19-(4*k) -: 4] + 55 : C_mat[ 19-(4*k) -: 4] + 48;
		data[141] = " ";
		k = k + 1;
		if (k == 5) data_done = 1;
		else data_done = 0;
	end 
end
// ---------------------------- LCD --------------------------------------
always @(posedge clk) begin
  if (~reset_n) begin
    row_A = "SD card cannot  ";
    row_B = "be initialized! ";
  end else if (P_next == S_UART_PRIN) begin
    row_A <= "The answer has  ";
	row_B <= " been uploaded.  ";
  end else if (P == S_BTN1_WAIT) begin
    row_A <= "Hit BTN1 to read";
    row_B <= "the SD card ... ";
  end
end
// ----------------- UART print data FSM -------------------
always @(posedge clk) begin
  if (~reset_n) Q <= S_UART_IDLE;
  else Q <= Q_next;
end
always @(*) begin // FSM next-state logic
  case (Q)
    S_UART_IDLE: // wait for button click
      if (print_message == 1 && uart_cnt <= 149) Q_next = S_UART_WAIT;
      else Q_next = S_UART_IDLE;
    S_UART_WAIT: // wait for the transmission of current data byte begins
      if (is_transmitting == 1) Q_next = S_UART_SEND;
      else Q_next = S_UART_WAIT;
    S_UART_SEND: // wait for the transmission of current data byte finishes
      if (is_transmitting == 0) Q_next = S_UART_INCR; // transmit next character
      else Q_next = S_UART_SEND;
    S_UART_INCR:
      if (tx_byte == 8'b0) Q_next = S_UART_IDLE; // data transmission ends
      else Q_next = S_UART_WAIT;
  endcase
end
// ------------------------------ UART Signal ----------------------------
assign transmit = print_message;
assign tx_byte = data[uart_cnt];
// ------------------------------ UART data counter ----------------------
always @(posedge clk) begin
  if (~reset_n) uart_cnt <= 0;
  if (Q_next == S_UART_INCR) uart_cnt <= uart_cnt + 1;
end
//---------------------------------------------------------------------------------
endmodule
