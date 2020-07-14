`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2017/09/26 15:03:31
// Design Name: 
// Module Name: lab3
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


module lab3(
  input  clk,            // System clock at 100 MHz
  input  reset_n,        // System reset signal, in negative logic
  input  [3:0] usr_btn,  // Four user pushbuttons
  output [3:0] usr_led   // Four yellow LEDs
);

    reg pwm;
    reg [3:0] counter=4'b0000;
    reg [3:0] btn_chk=4'b0000;
    integer state=4;
    integer pwm_cnt=0;
    
    assign usr_led[0]= counter[0] && pwm;
    assign usr_led[1]= counter[1] && pwm;
    assign usr_led[2]= counter[2] && pwm;
    assign usr_led[3]= counter[3] && pwm;
    
    
    always@(posedge clk)
    begin
    //-----------------------Reset----------------------------//    
    if(!reset_n)
    begin
            btn_chk=4'b0000;
            counter=4'b0001;
            pwm_cnt=0;
            state=2;
    end
    else 
    begin 
    //-----------------------Button_1 > cnt ++ ----------------------------//
        if(btn_chk[1] && !usr_btn[1])
                btn_chk[1]=1'b0;             // Reset debounce trigger
        if(!btn_chk[1])                      // First debounce trigger in
        begin
          if(usr_btn[1])                     // Check 1st button state
          begin
             #40000000;                      // Wait for time to second debounce
             if(usr_btn[1])                  // Check 2nd button state
             begin
                if(counter!=4'b0111)
                   counter=counter+4'b0001;                 
                btn_chk[1]=1'b1;             // Wait for reset
             end
          end 
        end
     //-----------------------Button_0 > cnt -- ----------------------------//
        if(btn_chk[0] && !usr_btn[0])
                btn_chk[0]=1'b0;
        if(!btn_chk[0])
        begin
            if(usr_btn[0])
            begin
            #40000000;
                if(usr_btn[0])
                begin
                    if(counter!=4'b0000 && counter!=4'b1000)
                            counter=counter-4'b0001;
                    else if(counter==4'b0000)
                            counter=4'b1111;                
                    btn_chk[0]=1'b1;   
                end   
            end
        end
        //-----------------------Button_3 > state ++ ----------------------------//
        if(btn_chk[3] && !usr_btn[3])
                btn_chk[3]=1'b0;
        if(!btn_chk[3])
        begin
            if(usr_btn[3])
            begin
            #40000000;
            if(usr_btn[3])
                if(state!=4)
                    state=state+1;
            btn_chk[3]=1'b1;
            end
        end
        //-----------------------Button_2 > state -- ----------------------------//
        if(btn_chk[2] && !usr_btn[2])
                btn_chk[2]=1'b0;
        if(!btn_chk[2])
        begin
             if(usr_btn[2])
             begin
             #40000000;
                if(usr_btn[2])
                     if(state!=0)
                         state=state-1;
                 btn_chk[2]=1'b1;
             end
         end

//-----------------------PWM----------------------------//
     pwm_cnt=pwm_cnt+1;
     if(pwm_cnt==1000000)
            pwm_cnt=0;
     case(state)
         0:  pwm=(pwm_cnt>50000)?1'b0:1'b1; 
         1:  pwm=(pwm_cnt>250000)?1'b0:1'b1;
         2:  pwm=(pwm_cnt>500000)?1'b0:1'b1;
         3:  pwm=(pwm_cnt>750000)?1'b0:1'b1; 
         4:  pwm=1'b1; 
         default: pwm=1'b0; 
     endcase
     
     end
end

endmodule