`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Dept. of CS, National Chiao Tung University
// Engineer: Chun-Jen Tsai
// 
// Create Date: 2017/10/16 14:21:33
// Design Name: 
// Module Name: lab5
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


module lab5(
  input clk,
  input reset_n,
  input [3:0] usr_btn,
  output [3:0] usr_led,
  output LCD_RS,
  output LCD_RW,
  output LCD_E,
  output [3:0] LCD_D
);

// turn off all the LEDs
assign usr_led = 4'b0000;

wire btn_level, btn_pressed;
reg pre_btn = 0;
reg [127:0] row_A = " Lab05 0516320  "; // Initialize the text of the first row. 
reg [127:0] row_B = "Char LCD Control"; // Initialize the text of the second row.

localparam [10:0] N = 1023;
localparam [5:0] R_N = 32;
localparam [2:0] check = 0, t_init = 1, minus = 2, count = 3, done = 4, ini = 5; 
reg [2:0] state = ini;
reg [0:1023] prime;
reg [7:0] cnt;
reg [10:0] idx,jdx;
reg [7:0] out_idx1 = 0,out_idx2 = 1;
reg [10:0] data [0:200];
reg [10:0] put1,put2;
reg waitt = 0;
reg [7:0] HL1,LH1,LL1;
reg [7:0] CH1,CL1;
reg [7:0] nout1 [2:0];
reg [7:0] cout1 [1:0];
reg [7:0] HL2,LH2,LL2;
reg [7:0] CH2,CL2;
reg [7:0] nout2 [2:0];
reg [7:0] cout2 [1:0];
reg rdown = 1;
reg flag;
integer i;
reg [25:0] counter = 0;


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

  

assign usr_led = 4'b0000;

always @(posedge clk)begin
    if (~reset_n)begin
        pre_btn = 0;
        rdown = 1;
    end
    else begin
        if(pre_btn && !usr_btn[3])begin
               pre_btn = 1'b0;
            end
        if(!pre_btn)begin
          if(usr_btn[3])begin
             #40000000;
             if(usr_btn[3])begin
                rdown = !rdown;
                pre_btn=1'b1;
             end
             end
        end
    end 
end

always @ (posedge clk) begin
    counter = counter + 1;
end

always @ (posedge clk) begin
	case(state)
	    ini: begin
	    
	    for(i=0;i<=N;i=i+1) prime[i]=1;
        prime[0]=0;
        prime[1]=0;
        idx = 2;
        cnt = 0;
        state = check;
	    
	    end
        check:begin
            if(idx>R_N)begin
                state = count;
                idx=0;
            end
            else
                if( prime[idx]==1 ) state = t_init;
                else begin
                    idx = idx + 1;
                    state = check;
                end
        end
        t_init: begin
            jdx = 2*idx;
            state = minus;
        end
        minus: begin
            if(jdx>N) begin
                idx = idx +1 ;
                state = check;
            end
            else begin
                prime[jdx] = 0;
                jdx = jdx + idx;
                state = minus;
            end
        end
        count: begin
            if(idx>N)state = done;
            else begin
                if(prime[idx]==1)begin
                cnt = cnt + 1;
                data[cnt] = idx;
                end
                idx = idx + 1;
                state = count;
                end
        end
        done:begin
            state = done;
        end
    endcase
end

always @(posedge clk)begin
    if (~reset_n) begin
        // Initialize the text when the user hit the reset button
        out_idx1 = 0;
        out_idx2 = 1;
    end
    else begin
    if(state == done && counter == 0)begin      
        if(rdown)begin
            out_idx1 = out_idx1 + 1;
            if(out_idx1 > cnt) out_idx1 = 1;
            put1 = data[out_idx1];
        end
        else begin
            out_idx1 = out_idx1 - 1;
            if(out_idx1 < 1) out_idx1 = cnt;
            put1 = data[out_idx1];
        end        
        
        HL1[7:0] = { 5'b00000,put1[10:8] }; LH1 [7:0] = { 4'b0000,put1[7:4] }; LL1 [7:0] = { 4'b0000,put1[3:0] };
        if( HL1<10 ) nout1[2] = HL1 + 8'h30 ;  else nout1[2] = HL1 + 8'h37 ;
        if( LH1<10 ) nout1[1] = LH1 + 8'h30 ;  else nout1[1] = LH1 + 8'h37 ;
        if( LL1<10 ) nout1[0] = LL1 + 8'h30 ;  else nout1[0] = LL1 + 8'h37 ;
        
        CH1[7:0] = { 4'b0000,out_idx1[7:4] }; CL1 [7:0] = { 4'b0000,out_idx1[3:0] };
        if( CH1<10 ) cout1[1] = CH1 + 8'h30 ;  else cout1[1] = CH1 + 8'h37 ;
        if( CL1<10 ) cout1[0] = CL1 + 8'h30 ;  else cout1[0] = CL1 + 8'h37 ;
        
        if(rdown)begin
            out_idx2 = out_idx2 + 1;
            if(out_idx2 > cnt) out_idx2 = 1;
            put2 = data[out_idx2];
        end
        else begin
            out_idx2 = out_idx2 - 1;
            if(out_idx2 < 1) out_idx2 = cnt;
            put2 = data[out_idx2];
        end   
        
		HL2[7:0] = { 5'b00000,put2[10:8] }; LH2 [7:0] = { 4'b0000,put2[7:4] }; LL2 [7:0] = { 4'b0000,put2[3:0] };
        if( HL2<10 ) nout2[2] = HL2 + 8'h30 ;  else nout2[2] = HL2 + 8'h37 ;
        if( LH2<10 ) nout2[1] = LH2 + 8'h30 ;  else nout2[1] = LH2 + 8'h37 ;
        if( LL2<10 ) nout2[0] = LL2 + 8'h30 ;  else nout2[0] = LL2 + 8'h37 ;
        
        CH2[7:0] = { 4'b0000,out_idx2[7:4] }; CL2 [7:0] = { 4'b0000,out_idx2[3:0] };
        if( CH2<10 ) cout2[1] = CH2 + 8'h30 ;  else cout2[1] = CH2 + 8'h37 ;
        if( CL2<10 ) cout2[0] = CL2 + 8'h30 ;  else cout2[0] = CL2 + 8'h37 ;
        
        row_A <= {"Prime #",cout1[1],cout1[0]," is ",nout1[2],nout1[1],nout1[0]};
        row_B <= {"Prime #",cout2[1],cout2[0]," is ",nout2[2],nout2[1],nout2[0]};
        
        end
    end

end



endmodule
