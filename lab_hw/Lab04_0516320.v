`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of CS, National Chiao Tung University
// Engineer: Chun-Jen Tsai
// 
// Create Date: 2017/04/27 15:06:57
// Design Name: UART I/O example for Arty
// Module Name: lab4
// Project Name: 
// Target Devices: Xilinx FPGA @ 100MHz
// Tool Versions: 
// Description: 
// 
// The parameters for the UART controller are 9600 baudrate, 8-N-1-N
//
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module lab4(
  input  clk,
  input  reset_n,
  input  [3:0] usr_btn,
  output [3:0] usr_led,
  input  uart_rx,
  output uart_tx
);

localparam [3:0] S_MAIN_INIT = 0, S_MAIN_FIRQ = 1,
                 S_MAIN_WAIT_KEY = 2, S_MAIN_SECQ = 3,
                 S_INTERF = 4 , S_MAIN_ENTIN1=5 ,
                 S_INTERS = 6 , S_MAIN_ENTIN2=7 ,
                 S_MAIN_WAIT_KEY2 = 8 , S_MAIN_ANS = 9,
                 S_MAIN_WAIT_ANS = 10;      
localparam [1:0] S_UART_IDLE = 0, S_UART_WAIT = 1,
                 S_UART_SEND = 2, S_UART_INCR = 3;

// declare system variables
wire print_enable, print_done;
reg [7:0] send_counter;
reg [3:0] P, P_next;
reg [1:0] Q, Q_next;
reg [23:0] init_counter;

// declare UART signals
wire transmit;
wire received;
wire [7:0] rx_byte;
wire enter_pressed,num_pressed;
reg  [7:0] rx_temp;
reg  in1,in2;
reg  [7:0] num1[0:4];
reg  [7:0] num2[0:4];
reg  [15:0] numa,numb;
wire [7:0] tx_byte;
wire is_receiving;
wire is_transmitting;
wire recv_error;

// GCD 
reg [15:0] gcd;
reg [2:0] state;
reg [15:0] ra,rb;
localparam init = 3'h0;
localparam start = 3'h1;
localparam check = 3'h2;
localparam comp = 3'h3;
localparam lastend = 3'h4;
localparam ended = 3'h5;
reg [7:0] GCDout [5:0];
reg [7:0] HH,HL,LH,LL;
reg GCD_done;
reg All_done;


/* The UART device takes a 100MHz clock to handle I/O at 9600 baudrate */
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

// Initializes some strings.
// System Verilog has an easier way to initialize an array,
// but we are using Verilog 2005 :(
//
localparam MEM_SIZE = 110;
localparam FIRQ_STR = 0;
localparam SECQ_STR = 32;
localparam ANS_STR = 64;
localparam INTER_STR = 88;
reg [7:0] data[0:MEM_SIZE-1];


initial begin
  { data[ 0], data[ 1], data[ 2], data[ 3], data[ 4], data[ 5], data[ 6], data[ 7],
    data[ 8], data[ 9], data[10], data[11], data[12], data[13], data[14], data[15],
    data[16], data[17], data[18], data[19], data[20], data[21], data[22], data[23],
    data[24], data[25], data[26], data[27], data[28], data[29], data[30], data[31] }
  <= { 8'h0D, 8'h0A, "Enter first decimal number : ", 8'h00 };

  { data[32], data[33], data[34], data[35], data[36], data[37], data[38], data[39],
    data[40], data[41], data[42], data[43], data[44], data[45], data[46], data[47],
	data[48], data[49], data[50], data[51], data[52], data[53], data[54], data[55],
	data[56], data[57], data[58], data[59], data[60], data[61], data[62], data[63] }
  <= { 8'h0D, 8'h0A, "Enter second decimal number: ", 8'h00 };
  
  
  
  { data[64], data[65], data[66], data[67], data[68], data[69], data[70], data[71],
    data[72], data[73], data[74], data[75], data[76], data[77], data[78], data[79] }
  <= { 8'h0D, 8'h0A, "The GCD is :  " };
  
  { data[80], data[81], data[82], data[83], data[84], data[85], data[86], data[87] }
  <= { "0","x","0","0","0","0" , 8'h0A, 8'h00 };
  
  
  { data[88], data[89], data[90] }
  <= { 8'h1C, 8'h00, 8'h00  };
  
  { num1[0], num1[1], num1[2], num1[3], num1[4],
    num2[0], num2[1], num2[2], num2[3], num2[4]  }
  <= { 8'h1C,8'h1C,8'h1C,8'h1C,8'h1C ,8'h1C,8'h1C,8'h1C,8'h1C,8'h1C};
  
    ra <= 16'h0;
    rb <= 16'h0;
    gcd <= 16'h0;
    state <= init;
    GCD_done <= 0 ;
    All_done <= 0 ;
    

  
end

// Combinational I/O logics
assign usr_led = usr_btn;


// ------------------------------------------------------------------------
// Main FSM that reads the UART input and triggers
// the output of the string "Hello, World!".
always @(posedge clk) begin
  if (~reset_n) P <= S_MAIN_INIT;
  else P <= P_next;
end

always @(*) begin // FSM next-state logic
  case (P)
    S_MAIN_INIT: // Delay 10 us.
     begin    
	   if (init_counter < 1000) P_next = S_MAIN_INIT;
	   else P_next = S_MAIN_FIRQ;
	 end
    S_MAIN_FIRQ: // Print the First message.
      if (print_done) P_next = S_MAIN_WAIT_KEY;
      else P_next = S_MAIN_FIRQ;
    S_MAIN_WAIT_KEY: // wait for < Enter or number > key.
      if (enter_pressed) P_next = S_MAIN_ENTIN1;
      else if (num_pressed) P_next = S_INTERF;
      else P_next = S_MAIN_WAIT_KEY;
    S_INTERF:
      if (print_done) P_next = S_MAIN_WAIT_KEY;
      else P_next = S_INTERF;
    S_MAIN_ENTIN1: // Delay 10 us to Second message.
      if (init_counter < 1000) P_next = S_MAIN_SECQ;
      else P_next = S_MAIN_ENTIN1;
    S_MAIN_SECQ: // Print the Second message.
      if (print_done) P_next = S_MAIN_WAIT_KEY2;
      else P_next = S_MAIN_SECQ;
    S_MAIN_WAIT_KEY2: // wait for < Enter or number > key.
      if (enter_pressed) P_next = S_MAIN_ENTIN2;
      else if (num_pressed) P_next = S_INTERS;
      else P_next = S_MAIN_WAIT_KEY2;
    S_INTERS:
      if (print_done) P_next = S_MAIN_WAIT_KEY2;
      else P_next = S_INTERS;
    S_MAIN_ENTIN2: // Delay 10 us to Final message.
      if (init_counter < 1000) P_next = S_MAIN_WAIT_ANS;
      else P_next = S_MAIN_ENTIN2;
    S_MAIN_WAIT_ANS: // Calculate answer .
      if (All_done) P_next = S_MAIN_ANS;
      else P_next = S_MAIN_WAIT_ANS;
    S_MAIN_ANS:
      if (print_done) P_next = S_MAIN_INIT;
      else P_next = S_MAIN_ANS;
  endcase
end

// FSM output logics: print string control signals.
assign print_enable = (P == S_MAIN_INIT && P_next == S_MAIN_FIRQ) ||
                      (P == S_MAIN_WAIT_KEY && P_next == S_INTERF) ||
                      (P == S_MAIN_WAIT_KEY2 && P_next == S_INTERS) ||
                      (P == S_MAIN_ENTIN1 && P_next == S_MAIN_SECQ)||
                      (P == S_MAIN_WAIT_ANS && P_next == S_MAIN_ANS);
assign print_done = (tx_byte == 8'h0);

// Initialization counter.
always @(posedge clk) begin
  if (P == S_MAIN_INIT || P == S_MAIN_ENTIN1 || P == S_MAIN_ENTIN2) init_counter <= init_counter + 1;
  else init_counter <= 0;
end
// End of the FSM of the print string controller
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
// FSM of the controller to send a string to the UART.
always @(posedge clk) begin
  if (~reset_n) Q <= S_UART_IDLE;
  else Q <= Q_next;
end

always @(*) begin // FSM next-state logic
  case (Q)
    S_UART_IDLE: // wait for the print_string flag
      if (print_enable) Q_next = S_UART_WAIT;
      else Q_next = S_UART_IDLE;
    S_UART_WAIT: // wait for the transmission of current data byte begins
      if (is_transmitting == 1) Q_next = S_UART_SEND;
      else Q_next = S_UART_WAIT;
    S_UART_SEND: // wait for the transmission of current data byte finishes
      if (is_transmitting == 0) Q_next = S_UART_INCR; // transmit next character
      else Q_next = S_UART_SEND;
    S_UART_INCR:
      if (print_done) Q_next = S_UART_IDLE; // string transmission ends
      else Q_next = S_UART_WAIT;
  endcase
end

// FSM output logics
assign transmit = (Q_next == S_UART_WAIT || print_enable);
assign tx_byte = data[send_counter];

// UART send_counter control circuit
always @(posedge clk) begin
  case (P_next)
    S_MAIN_INIT: send_counter = FIRQ_STR;
    S_MAIN_WAIT_KEY: send_counter = INTER_STR;
    S_MAIN_WAIT_KEY2: send_counter = INTER_STR;
    S_MAIN_ENTIN1: send_counter = SECQ_STR;
    S_MAIN_ENTIN2: send_counter = ANS_STR;
    default: send_counter = send_counter + (Q_next == S_UART_INCR);
  endcase
end
// End of the FSM of the print string controller
// ------------------------------------------------------------------------

assign enter_pressed = (rx_temp == 8'h0D);
assign num_pressed = (rx_temp >= 8'h30 ) && (rx_temp <= 8'h39);

// ------------------------------------------------------------------------
// The following logic stores the UART input in a temporary buffer.
// The input character will stay in the buffer for one clock cycle.
always @(posedge clk) begin
    if(P==S_MAIN_INIT)begin
        { num1[0], num1[1], num1[2], num1[3], num1[4],
          num2[0], num2[1], num2[2], num2[3], num2[4]  }
        <= { 8'h1C,8'h1C,8'h1C,8'h1C,8'h1C ,8'h1C,8'h1C,8'h1C,8'h1C,8'h1C};
        numa = 16'h0; numb = 16'h0; 
   
        end
    if(P==S_MAIN_ENTIN2)begin
                    if(num2[0]==8'h1C) num2[0]<=data[89];
                    else begin 
                         if(num2[1]==8'h1C) begin num2[1]<=num2[0]; num2[0]<=data[89]; end
                         else begin
                              if(num2[2]==8'h1C) begin num2[2]<=num2[1]; num2[1]<=num2[0]; num2[0]<=data[89]; end
                              else begin
                                   if(num2[3]==8'h1C) begin num2[3]<=num2[2];num2[2]<=num2[1]; num2[1]<=num2[0]; num2[0]<=data[89]; end
                                   else begin
                                        if(num2[4]==8'h1C) begin num2[4]<=num2[3];num2[3]<=num2[2];num2[2]<=num2[1]; num2[1]<=num2[0]; num2[0]<=data[89]; end
                                        end
                                   end
                              end
                         end
                                     
                     
      if(num1[4]==8'h1C) num1[4]="0";
      if(num1[3]==8'h1C) num1[3]="0";
      if(num1[2]==8'h1C) num1[2]="0";
      if(num1[1]==8'h1C) num1[1]="0";
      if(num1[0]==8'h1C) num1[0]="0";
      if(num2[4]==8'h1C) num2[4]="0";
      if(num2[3]==8'h1C) num2[3]="0";
      if(num2[2]==8'h1C) num2[2]="0";
      if(num2[1]==8'h1C) num2[1]="0";
      if(num2[0]==8'h1C) num2[0]="0";
      
      numa[15:0] = ( num1[4]-8'h30 )*10000 + ( num1[3]-8'h30 )*1000 + ( num1[2]-8'h30 )*100 + ( num1[1]-8'h30 )*10 + ( num1[0]-8'h30 );
      numb[15:0] = ( num2[4]-8'h30 )*10000 + ( num2[3]-8'h30 )*1000 + ( num2[2]-8'h30 )*100 + ( num2[1]-8'h30 )*10 + ( num2[0]-8'h30 );
      
      end
      
  rx_temp <= (received)? rx_byte : 8'h00;
  if (num_pressed) begin
    data[89] = rx_temp;
    if( P == S_MAIN_WAIT_KEY )begin
            if(num1[0]==8'h1C) num1[0]<=data[89];
            else begin 
                 if(num1[1]==8'h1C) begin num1[1]<=num1[0]; num1[0]<=data[89]; end
                 else begin
                      if(num1[2]==8'h1C) begin num1[2]<=num1[1]; num1[1]<=num1[0]; num1[0]<=data[89]; end
                      else begin
                           if(num1[3]==8'h1C) begin num1[3]<=num1[2];num1[2]<=num1[1]; num1[1]<=num1[0]; num1[0]<=data[89]; end
                           else begin
                                if(num1[4]==8'h1C) begin num1[4]<=num1[3];num1[3]<=num1[2];num1[2]<=num1[1]; num1[1]<=num1[0]; num1[0]<=data[89]; end
                                end
                           end
                      end
                 end              
    end
    else if( P == S_MAIN_WAIT_KEY2 ) begin
        if(num2[0]==8'h1C) num2[0]<=data[89];
        else begin 
             if(num2[1]==8'h1C) begin num2[1]<=num2[0]; num2[0]<=data[89]; end
             else begin
                  if(num2[2]==8'h1C) begin num2[2]<=num2[1]; num2[1]<=num2[0]; num2[0]<=data[89]; end
                  else begin
                       if(num2[3]==8'h1C) begin num2[3]<=num2[2];num2[2]<=num2[1]; num2[1]<=num2[0]; num2[0]<=data[89]; end
                       else begin
                            if(num2[4]==8'h1C) begin num2[4]<=num2[3];num2[3]<=num2[2];num2[2]<=num2[1]; num2[1]<=num2[0]; num2[0]<=data[89]; end
                            end
                       end
                  end
             end        
         end
  end
end
// ------------------------------------------------------------------------


always @ (posedge clk)
begin
    if(P==S_MAIN_INIT) state = init;
    case(state)
        init:
            begin
                ra <= 16'h0;
                rb <= 16'h0;
                gcd <= 16'h0;
                state = ( P == S_MAIN_WAIT_ANS ) ? start : init;
                GCD_done <= 0 ;
            end           
        start: //status 0
                begin
                    ra <= numa;
                    rb <= numb;
                    state <= check;
                end
        check:  //Status 1
                begin
                    if ((ra == 16'h0) || (ra == 16'h0))
                    begin
                        state <= lastend;
                    end
                    else begin
                        state <= comp;
                    end
                end
        comp: //status 2
                begin
                    if(ra > rb) //Compare ra and rb
                    begin
                        ra = ra - rb;
                        if((ra < 16'h0) || (rb < 16'h0)) //Compare ra and rb and if either has become 0
                        begin
                            state <= lastend;
                        end
                        else begin
                            state <= comp;
                            gcd <= ra;
                        end
                    end
                    else if (rb > ra) //Compare ra and rb
                    begin
                        rb = rb - ra;
                        if((ra < 16'h0) || (rb < 16'h0))//Compare ra and rb and if either has become 0
                        begin
                            state <= lastend;
                        end
                        else begin
                            state <= comp;
                            gcd <= ra;
                        end
                    end
                    else if(ra == rb) //Finally gcd found ra == rb
                        begin
                            gcd <= ra;
                            state <= ended;
                        end
                end
        lastend: //status 3
                        begin
                            gcd <= 16'h0001;
                            state <= ended;
                        end
        ended:
            begin
                gcd <= gcd;
                ra <= ra;
                rb <= rb;
                state <= state;
                GCD_done <= 1;
            end
    endcase
end


always @(posedge clk)
begin
    if(GCD_done)begin
        GCDout[5]="0";
        GCDout[4]="x";
        HH [7:0] = { 4'b0000,gcd[15:12] } ; HL [7:0] = { 4'b0000,gcd[11:8] }; LH [7:0] = { 4'b0000,gcd[7:4] }; LL [7:0] = { 4'b0000,gcd[3:0] };
        if( HH<9 ) GCDout[3] = HH + 8'h30 ;  else GCDout[3] = HH + 8'h37 ;
        if( HL<9 ) GCDout[2] = HL + 8'h30 ;  else GCDout[2] = HL + 8'h37 ;
        if( LH<9 ) GCDout[1] = LH + 8'h30 ;  else GCDout[1] = LH + 8'h37 ;
        if( LL<9 ) GCDout[0] = LL + 8'h30 ;  else GCDout[0] = LL + 8'h37 ;
        
        { data[64], data[65], data[66], data[67], data[68], data[69], data[70], data[71],
          data[72], data[73], data[74], data[75], data[76], data[77], data[78], data[79] }
        <= { 8'h0D, 8'h0A, "The GCD is :  " };
        
        { data[80], data[81], data[82], data[83], data[84], data[85], data[86], data[87] }
        <= { GCDout[5], GCDout[4], GCDout[3], GCDout[2], GCDout[1], GCDout[0], 8'h0A, 8'h00 };
        All_done=1;
    end     
    else   begin
        { GCDout[5], GCDout[4], GCDout[3], GCDout[2], GCDout[1], GCDout[0] }
        <= { "0x0000" };
        HH = 8'h0; LH = 8'h0; HL = 8'h0; LL = 8'h0;
        All_done <= 0 ;
    end        
end

endmodule
