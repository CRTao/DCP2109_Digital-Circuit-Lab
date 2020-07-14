`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of CS, National Chiao Tung University
// Engineer: Chun-Jen Tsai
// 
// Create Date: 2017/12/11 14:21:33
// Design Name: 
// Module Name: lab08
// Project Name: 
// Target Devices: Xilinx FPGA @ 100MHz 
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


module Lab08(
  input clk,
  input reset_n,
  input [3:0] usr_btn,
  output [3:0] usr_led,
  output LCD_RS,
  output LCD_RW,
  output LCD_E,
  output [3:0] LCD_D
);

assign usr_led = 4'b0000;


wire btn_level, btn_pressed;
reg prev_btn_level;
reg [127:0] row_A = "Press BTN3 to   ";
reg [127:0] row_B = "Start Cracking..";

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
  .btn_input(usr_btn[3]),
  .btn_output(btn_level)
);

//---------------reg---------------
//
reg [4:0] r [63:0];
reg [31:0] k [63:0];

initial begin
  { r[ 0], r[ 1], r[ 2], r[ 3], r[ 4], r[ 5], r[ 6], r[ 7], r[ 8], r[ 9], r[10], r[11], r[12], r[13], r[14], r[15],
    r[16], r[17], r[18], r[19], r[20], r[21], r[22], r[23], r[24], r[25], r[26], r[27], r[28], r[29], r[30], r[31],
    r[32], r[33], r[34], r[35], r[36], r[37], r[38], r[39], r[40], r[41], r[42], r[43], r[44], r[45], r[46], r[47],
    r[48], r[49], r[50], r[51], r[52], r[53], r[54], r[55], r[56], r[57], r[58], r[59], r[60], r[61], r[62], r[63] }  
    <= {5'd7, 5'd12, 5'd17, 5'd22, 5'd7, 5'd12, 5'd17, 5'd22, 5'd7, 5'd12, 5'd17, 5'd22, 5'd7, 5'd12, 5'd17, 5'd22,
        5'd5,  5'd9, 5'd14, 5'd20, 5'd5,  5'd9, 5'd14, 5'd20, 5'd5,  5'd9, 5'd14, 5'd20, 5'd5,  5'd9, 5'd14, 5'd20,
        5'd4, 5'd11, 5'd16, 5'd23, 5'd4, 5'd11, 5'd16, 5'd23, 5'd4, 5'd11, 5'd16, 5'd23, 5'd4, 5'd11, 5'd16, 5'd23,
        5'd6, 5'd10, 5'd15, 5'd21, 5'd6, 5'd10, 5'd15, 5'd21, 5'd6, 5'd10, 5'd15, 5'd21, 5'd6, 5'd10, 5'd15, 5'd21};
      
  { k[ 0], k[ 1], k[ 2], k[ 3], k[ 4], k[ 5], k[ 6], k[ 7], k[ 8], k[ 9], k[10], k[11], k[12], k[13], k[14], k[15],
    k[16], k[17], k[18], k[19], k[20], k[21], k[22], k[23], k[24], k[25], k[26], k[27], k[28], k[29], k[30], k[31],
    k[32], k[33], k[34], k[35], k[36], k[37], k[38], k[39], k[40], k[41], k[42], k[43], k[44], k[45], k[46], k[47],
    k[48], k[49], k[50], k[51], k[52], k[53], k[54], k[55], k[56], k[57], k[58], k[59], k[60], k[61], k[62], k[63] }
     = {32'hd76aa478, 32'he8c7b756, 32'h242070db, 32'hc1bdceee,
        32'hf57c0faf, 32'h4787c62a, 32'ha8304613, 32'hfd469501,
        32'h698098d8, 32'h8b44f7af, 32'hffff5bb1, 32'h895cd7be,
        32'h6b901122, 32'hfd987193, 32'ha679438e, 32'h49b40821,
        32'hf61e2562, 32'hc040b340, 32'h265e5a51, 32'he9b6c7aa,
        32'hd62f105d, 32'h02441453, 32'hd8a1e681, 32'he7d3fbc8,
        32'h21e1cde6, 32'hc33707d6, 32'hf4d50d87, 32'h455a14ed,
        32'ha9e3e905, 32'hfcefa3f8, 32'h676f02d9, 32'h8d2a4c8a,
        32'hfffa3942, 32'h8771f681, 32'h6d9d6122, 32'hfde5380c,
        32'ha4beea44, 32'h4bdecfa9, 32'hf6bb4b60, 32'hbebfbc70,
        32'h289b7ec6, 32'heaa127fa, 32'hd4ef3085, 32'h04881d05,
        32'hd9d4d039, 32'he6db99e5, 32'h1fa27cf8, 32'hc4ac5665,
        32'hf4292244, 32'h432aff97, 32'hab9423a7, 32'hfc93a039,
        32'h655b59c3, 32'h8f0ccc92, 32'hffeff47d, 32'h85845dd1,
        32'h6fa87e4f, 32'hfe2ce6e0, 32'ha3014314, 32'h4e0811a1,
        32'hf7537e82, 32'hbd3af235, 32'h2ad7d2bb, 32'heb86d391 };
end

localparam [3:0]  S_MAIN_INIT = 4'b0000, S_MAIN_SMD5 = 4'b0001,
                  S_MAIN_16WD = 4'b0010, S_MAIN_FGCA = 4'b0011,
					        S_MAIN_SHIF = 4'b0100, S_ADD2_ANSW = 4'b0101,
					        S_ANS2_HASH = 4'b0110, S_HASH2_128 = 4'b0111,
                  S_COMP_PWHS = 4'b1000, S_MAIN_DONE = 4'b1001;

reg  [3:0] P, P_next;

reg  [0:511] msg = 0;
reg  [0:31] h0 = 0,h1 = 0,h2 = 0,h3 = 0;
reg  [31:0] a,b,c,d;
reg  [31:0] f,g;
reg  [9:0] i;

reg  [0:127] passwd_hash = 128'hE9982EC5CA981BD365603623CF4B2277;
reg  [0:63] guess_num = 0;
reg  [0:127] guess_hash;

reg [19:0]clock_cnt;
reg [16:0]cnt2ms;

//--------------- FSM next-state logic-------------------

always @(posedge clk) begin
  if (~reset_n) 
    P <= S_MAIN_INIT;
  else 
    P <= P_next;
end

always @(*) begin
  case (P)
    S_MAIN_INIT:
      if (btn_pressed == 1) P_next = S_MAIN_SMD5;
	  else P_next=S_MAIN_INIT;
    S_MAIN_SMD5:
	    P_next=S_MAIN_16WD;
    S_MAIN_16WD:
      if(i<64) P_next=S_MAIN_FGCA;
      else P_next=S_ADD2_ANSW;
    S_MAIN_FGCA:
      P_next=S_MAIN_SHIF;
    S_MAIN_SHIF:
      P_next=S_MAIN_16WD;
    S_ADD2_ANSW:
      P_next=S_ANS2_HASH;
    S_ANS2_HASH:
      P_next=S_HASH2_128;
    S_HASH2_128:
      P_next=S_COMP_PWHS;
    S_COMP_PWHS:
      if(guess_hash==passwd_hash) P_next=S_MAIN_DONE;
      else P_next=S_MAIN_SMD5;
    S_MAIN_DONE:
      P_next=S_MAIN_DONE;
    default:
      P_next = S_MAIN_INIT;
  endcase
end
//-------------- Messege mod 448 ---------------- 
always @(posedge clk) begin
  if (~reset_n||P_next==S_MAIN_SMD5)
    msg<=0;
  else if(P==S_MAIN_SMD5) begin
    msg[0:63] <= guess_num;
	  msg[64:71] <= 128;
	  msg[448:455] <= 64;
  end
end
// --------------- Algorithm looping ------------------
always @(posedge clk) begin
  if (~reset_n || (P==S_MAIN_SMD5)) begin
	   a<=h0; b<=h1; c<=h2; d<=h3; i<=0;
    end
  else if( P==S_MAIN_16WD && P_next==S_MAIN_FGCA )begin
    if(i<16) begin
      f <= (b & c) | ((~b) & d);
        g <= i;
    end else if(i<32) begin
      f <= (d & b) | ((~d) & c);
        g <= 5*i + 1;
    end else if(i<48) begin
      f <= b ^ c ^ d;
        g <= 3*i + 5;
    end else if(i<64) begin
      f <= c ^ (b | (~d));
        g <= 7*i;
    end
  end 
  else if(P==S_MAIN_FGCA && P_next==S_MAIN_SHIF) begin
    if(i>15) g <= {28'b0,g[3:0]}; // msg index shifting
  end else if(P==S_MAIN_SHIF && P_next==S_MAIN_16WD) begin
    i<=i+1;
    d<=c;
    c<=b;
    a<=d;
    b<=b+(((a+f+k[i]+{msg[(32*g+24)+:8],msg[(32*g+16)+:8],msg[(32*g+8)+:8],msg[(32*g)+:8]})<<(r[i]))|
         ((a+f+k[i]+{msg[(32*g+24)+:8],msg[(32*g+16)+:8],msg[(32*g+8)+:8],msg[(32*g)+:8]})>>(32-r[i])));
  end
end
//------------- Stantard Hash ---------------
always @(posedge clk) begin
  if(~reset_n || P_next==S_MAIN_SMD5)begin
    h0 <= 32'h67452301;
    h1 <= 32'hefcdab89;
    h2 <= 32'h98badcfe;
    h3 <= 32'h10325476;
  end else if(P==S_ADD2_ANSW && P_next==S_ANS2_HASH)begin
    h0 <= h0+a;
    h1 <= h1+b;
    h2 <= h2+c;
    h3 <= h3+d;
  end
end
// ------------- Final Guess Hash ---------------
always @(posedge clk) begin
  if (~reset_n||P_next==S_MAIN_SMD5)
    guess_hash<=0;
  else if(P==S_ANS2_HASH&&P_next==S_HASH2_128) begin
    guess_hash[ 0:  7]<=h0[24:31];guess_hash[  8: 15]<=h0[16:23];guess_hash[ 16: 23]<=h0[8:15];guess_hash[ 24: 31]<=h0[0:7];
    guess_hash[32: 39]<=h1[24:31];guess_hash[ 40: 47]<=h1[16:23];guess_hash[ 48: 55]<=h1[8:15];guess_hash[ 56: 63]<=h1[0:7];
    guess_hash[64: 71]<=h2[24:31];guess_hash[ 72: 79]<=h2[16:23];guess_hash[ 80: 87]<=h2[8:15];guess_hash[ 88: 95]<=h2[0:7];
    guess_hash[96:103]<=h3[24:31];guess_hash[104:111]<=h3[16:23];guess_hash[112:119]<=h3[8:15];guess_hash[120:127]<=h3[0:7];
  end
end
// ------------ Guess Num ++ ---------------------
always @(posedge clk) begin
  if (~reset_n||P==S_MAIN_INIT)
	   guess_num<={ 8'h30, 8'h30, 8'h30, 8'h30, 8'h30, 8'h30, 8'h30, 8'h30};
  else if( P==S_COMP_PWHS && P_next==S_MAIN_SMD5 )
    if(guess_num[56:63]==8'h39)begin
      guess_num[56:63]<=8'h30;
      if(guess_num[48:55]==8'h39) begin
      guess_num[48:55]<=8'h30;
      if(guess_num[40:47]==8'h39) begin
        guess_num[40:47]<=8'h30;
        if(guess_num[32:39]==8'h39) begin
          guess_num[32:39]<=8'h30;
        if(guess_num[24:31]==8'h39) begin
          guess_num[24:31]<=8'h30;
          if(guess_num[16:23]==8'h39) begin
            guess_num[16:23]<=8'h30;
          if(guess_num[8:15]==8'h39) begin
            guess_num[8:15]<=8'h30;
            if(guess_num[0:7]==8'h39) begin
              guess_num[0:7]<=8'h30;
            end else
              guess_num[0:7]<=guess_num[0:7]+1;
          end else
            guess_num[8:15]<=guess_num[8:15]+1;
          end else
            guess_num[16:23]<=guess_num[16:23]+1;
        end else
          guess_num[24:31]<=guess_num[24:31]+1;
        end else
          guess_num[32:39]<=guess_num[32:39]+1;
      end else
        guess_num[40:47]<=guess_num[40:47]+1;
      end else
        guess_num[48:55]<=guess_num[48:55]+1;
    end else
      guess_num[56:63]<=guess_num[56:63]+1;
end

// -------------------- clock -------------------------
always @ (posedge clk) begin
  if (~reset_n||P==S_MAIN_INIT) begin
      cnt2ms <= 0;
      clock_cnt <= 0;
  end
  else if(P!=S_MAIN_DONE)begin
    if( cnt2ms < 100000 ) 
        cnt2ms <= cnt2ms + 1;
    else if(cnt2ms == 100000)begin
        cnt2ms <= 0;
        clock_cnt = clock_cnt + 1;        
    end
  end
end

//--------------------- Debounce ------------------------
always @(posedge clk) begin
  if (~reset_n)
    prev_btn_level <= 1;
  else
    prev_btn_level <= btn_level;
end
assign btn_pressed = (btn_level == 1 && prev_btn_level == 0);

//--------------------- LCD Display ---------------------
always @(posedge clk) begin
  if (~reset_n) begin
    row_A = "Press BTN3 to   ";
    row_B = "Start Cracking..";
  end else if (P!=S_MAIN_INIT) begin
    row_A <= {"Passwd: ",guess_num};
    row_B[127:80] <= {"Time: "};
    row_B[23:0] <= {" ms"};
    row_B[79:72]<=clock_cnt/1000000+48;
    row_B[71:64]<=clock_cnt/100000-clock_cnt/1000000*10+48;
    row_B[63:56]<=clock_cnt/10000-clock_cnt/100000*10+48;
    row_B[55:48]<=clock_cnt/1000-clock_cnt/10000*10+48;
    row_B[47:40]<=clock_cnt/100-clock_cnt/1000*10+48;
    row_B[39:32]<=clock_cnt/10-clock_cnt/100*10+48;
    row_B[31:24]<=clock_cnt/1-clock_cnt/10*10+48;
  end
end
//----------------------------------------------------------
endmodule