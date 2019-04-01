`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2017/12/12 14:48:55
// Design Name: 
// Module Name: seq
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
module cenrreg(in,out,set,reset,reVal,clock);//同步清零的寄存器
    parameter width = 8;//寄存器宽度
    input[width-1:0] in;//更新寄存器的输入
    input set;//更新寄存器的使能信号
    input reset;//复位信号
    input clock;//时钟
    output[width-1:0] out;//寄存器输出
    reg [width-1:0] out;
    input [width-1:0] reVal;
    always@(posedge clock)//时钟上升沿更新
    begin
        if(set)//如果设置为1
            out <= in;
        if(reset)//如果复位为1
            out <= reVal;
     end            
endmodule
//寄存器文件
module registerFile(srcA,srcB,valA,valB,dstE,valE,dstM,valM,reset,clock,rax,rcx,rdx,rbx,rsp,rbp,rsi,rdi,r8,r9,r10,r11,r12,r13,r14);
    //寄存器的id位宽，和寄存器的位宽
    parameter valSize = 64;
    parameter regAddr = 4;
    //规定寄存器参数
    parameter IRAX = 4'h0;
    parameter IRCX = 4'h1;
    parameter IRDX = 4'h2;
    parameter IRBX = 4'h3;
    parameter IRSP = 4'h4;
    parameter IRBP = 4'h5;
    parameter IRSI = 4'h6;
    parameter IRDI = 4'h7;
    parameter IR8 = 4'h8;
    parameter IR9 = 4'h9;
    parameter IRA = 4'ha;
    parameter IRB = 4'hb;
    parameter IRC = 4'hc;
    parameter IRD = 4'hd;
    parameter IRE = 4'he;
    parameter RNONE = 4'hf;
    //端口声明
    output[valSize-1:0]rax,rcx,rdx,rbx,rsp,rbp,rdi,rsi,r8,r9,r10,r11,r12,r13,r14;
    input[regAddr-1:0] srcA;
    input[regAddr-1:0] srcB;
    output[valSize-1:0]valA;
    output[valSize-1:0]valB;
    input[regAddr-1:0]dstE;
    input[regAddr-1:0]dstM;
    input[valSize-1:0]valE;
    input[valSize-1:0]valM;
    input reset;
    input clock;
    //声明寄存器输入数据端
    wire[valSize-1:0]raxData,rbxData,rcxData,rdxData,rspData,rbpData,rsiData,rdiData,r8Data,r9Data,r11Data,r12Data,r13Data,r14Data,r10Data;
    //声明寄存器写信号
    wire raxW,rbxW,rcxW,rdxW,rspW,rbpW,rsiW,rdiW,r8W,r9W,r10W,r11W,r12W,r13W,r14W;
    //调用cenrreg模块声明15个寄存器
    cenrreg #(64) raxReg(raxData,rax,raxW,reset,64'b0,clock);
    cenrreg #(64) rbxReg(rbxData,rbx,rbxW,reset,64'b0,clock);
    cenrreg #(64) rcxReg(rcxData,rcx,rcxW,reset,64'b0,clock);
    cenrreg #(64) rdxReg(rdxData,rdx,rdxW,reset,64'b0,clock);
    cenrreg #(64) rspReg(rspData,rsp,rspW,reset,64'b1000,clock);
    cenrreg #(64) rbpReg(rbpData,rbp,rbpW,reset,64'b0,clock);
    cenrreg #(64) rsiReg(rsiData,rsi,rsiW,reset,64'b0,clock);
    cenrreg #(64) rdiReg(rdiData,rdi,rdiW,reset,64'b0,clock);
    cenrreg #(64) r8Reg(r8Data,r8,r8W,reset,64'b0,clock);
    cenrreg #(64) r9Reg(r9Data,r9,r9W,reset,64'b0,clock);
    cenrreg #(64) raReg(r10Data,r10,r10W,reset,64'b0,clock);
    cenrreg #(64) rbReg(r11Data,r11,r11W,reset,64'b0,clock);
    cenrreg #(64) rcReg(r12Data,r12,r12W,reset,64'b0,clock);
    cenrreg #(64) rdReg(r13Data,r13,r13W,reset,64'b0,clock);
    cenrreg #(64) reReg(r14Data,r14,r14W,reset,64'b0,clock);
    //选择valA的值
    assign valA = srcA==IRAX ? rax:
                  srcA==IRDX ? rdx:
                  srcA==IRCX ? rcx:
                  srcA==IRBX ? rbx:
                  srcA==IRSP ? rsp:
                  srcA==IRBP ? rbp:
                  srcA==IRSI ? rsi:
                  srcA==IRDI ? rdi:
                  srcA==IR8 ? r8:
                  srcA==IR9 ? r9:
                  srcA==IRA ? r10:
                  srcA==IRB ? r11:
                  srcA==IRC ? r12:
                  srcA==IRD ? r13:
                  srcA==IRE ? r14:
                  0;
    //选择valB的值
    assign valB = srcB==IRAX ? rax:
                  srcB==IRDX ? rdx:
                  srcB==IRCX ? rcx:
                  srcB==IRBX ? rbx:
                  srcB==IRSP ? rsp:
                  srcB==IRBP ? rbp:
                  srcB==IRSI ? rsi:
                  srcB==IRDI ? rdi:
                  srcB==IR8 ? r8:
                  srcB==IR9 ? r9:
                  srcB==IRA ? r10:
                  srcB==IRB ? r11:
                  srcB==IRC ? r12:
                  srcB==IRD ? r13:
                  srcB==IRE ? r14:
                  0;
    //对寄存器写的输入端赋值
    assign raxData = dstM == IRAX ? valM:valE;
    assign rdxData = dstM == IRDX ? valM:valE;
    assign rcxData = dstM == IRCX ? valM:valE;
    assign rbxData = dstM == IRBX ? valM:valE;
    assign rspData = dstM == IRSP ? valM : valE;
    assign rbpData = dstM == IRBP ? valM : valE;
    assign rsiData = dstM == IRSI ? valM : valE;
    assign rdiData = dstM == IRDI ? valM : valE;
    assign r8Data = dstM == IR8 ? valM : valE;
    assign r9Data = dstM == IR9 ? valM : valE;
    assign r10Data = dstM == IRA ? valM : valE;
    assign r11Data = dstM == IRB ? valM : valE;
    assign r12Data = dstM == IRC ? valM : valE;
    assign r13Data = dstM == IRD ? valM : valE;
    assign r14Data = dstM == IRE ? valM : valE;
    //对寄存器写的使能端赋值
     assign raxW = dstM == IRAX | dstE == IRAX;
       assign rcxW = dstM == IRCX | dstE == IRCX;
       assign rdxW = dstM == IRDX | dstE == IRDX;
       assign rbxW = dstM == IRBX | dstE == IRBX;
       assign rspW = dstM == IRSP | dstE == IRSP;
       assign rbpW = dstM == IRBP | dstE == IRBP;
       assign rsiW = dstM == IRSI | dstE == IRSI;
       assign rdiW = dstM == IRDI | dstE == IRDI;
       assign r8W = dstM == IR8 | dstE == IR8;
       assign r9W = dstM == IR9 | dstE == IR9;
       assign r10W = dstM == IRA | dstE == IRA;
       assign r11W = dstM == IRB | dstE == IRB;
       assign r12W = dstM == IRC | dstE == IRC;
       assign r13W = dstM == IRD | dstE == IRD;
       assign r14W = dstM == IRE | dstE == IRE;
endmodule
//随机访问存储器，模拟内存中的一个内存芯片
module ram(clock,addrA,wEnA,wDataA,rEnA,rDataA,addrB,wEnB,wDataB,rEnB,rDataB);
    //声明字长，芯片的字节数，字节的地址
    parameter WORDSIZE = 8;
    parameter WORDNUM = 512;
    parameter ADDRSIZE = 9;
    //输入输出端口声明
    //有两个既可以读又可以写的端口，地址输入分别为addrA，addrB，写的使能端分别为wEnA，wEnB，写的数据输入分别为wDataA,wDataB。
    //读的使能端分别为rEnA，rEnB，读的输出分别为rDataA，rDataB
    input clock;
    input [ADDRSIZE-1:0] addrA;
    input [WORDSIZE-1:0] wDataA;
    input wEnA;
    input rEnA;
    output reg[WORDSIZE-1:0] rDataA;
    input [ADDRSIZE-1:0] addrB;
    input [WORDSIZE-1:0] wDataB;
    input wEnB;
    input rEnB;
    output reg[WORDSIZE-1:0]rDataB;
    reg[WORDSIZE-1:0]mem[WORDNUM-1:0];
    //时钟下降沿时进行数据的读写
    always@(negedge clock)
    begin
        if(wEnA)
            begin
                mem[addrA] <= wDataA;
            end
        if(rEnA)
            begin
                rDataA <= mem[addrA];
            end
        end
    always@(negedge clock)
    begin
        if(wEnB)
            begin
                mem[addrB] <= wDataB;
            end
        if(rEnB)
            begin
                rDataB <= mem[addrB];
            end
        end
endmodule
//Fetch stage
//split the code into icode and ifun
//get the ifun and icode
module split(code,icode,ifun);
    input[7:0]code;
    output[3:0]icode;
    output[3:0]ifun;
    assign icode = code[7:4];
    assign ifun = code[3:0];
endmodule
//get the srcA,srcB and valC
module align(code,rA,rB,valC,needRegids);
    input[71:0]code;
    input needRegids;
    output[3:0]rA;
    output[3:0]rB;
    output[63:0]valC;
    assign rB = code[3:0];
    assign rA = code[7:4];
    assign valC = needRegids?code[71:8]: code[63:0];
endmodule
//用于计算valP的模块
//根据当前的pc，needRegids和needValC，计算出valP
module pc_increment(pc,needRegids,needValC,valP);
    input[63:0]pc;
    input needRegids;
    input needValC;
    output[63:0]valP;
    assign valP = pc+1+needRegids+8*needValC;
endmodule
//Execute Stage
//ALU
//可以进行and,sub,add,xor四中操作，得到valE和newCC
module alu(aluA,aluB,ifun,valE,newCC);
    //设定四种计算的编码
    parameter ALUADD = 4'h0;
    parameter ALUSUB = 4'h1;
    parameter ALUAND = 4'h2;
    parameter ALUXOR = 4'h3;
    input[63:0] aluA;
    input[63:0] aluB;
    input[3:0] ifun;
    output[63:0] valE;
    output[2:0] newCC;
    //计算valE
    assign valE = ifun==ALUADD ? aluA+aluB:
                  ifun==ALUSUB ? aluB-aluA:
                  ifun==ALUAND ? aluB&aluA:
                  aluB^aluA;
    //设置newCC
    assign newCC[2] = (valE==0);
    assign newCC[1] = valE[63];
    assign newCC[0] = (aluA[63]==aluB[63])&(valE[63]!=aluB[63])&(ifun==ALUADD) ? 1:
                      (aluA[63]!=aluB[63])&(aluB[63]!=valE[63])&(ifun==ALUSUB) ? 1:
                      0;
endmodule
//条件码寄存器，用于更新条件码
module CC(newCC,cc,setCC,reset,clock);
    output[2:0] cc;
    input[2:0] newCC;
    input setCC;
    input reset;
    input clock;
    
    cenrreg #(3)ccReg(newCC,cc,setCC,reset,3'b100,clock);
endmodule
// 计算Cnd，用于判断JXX和OMVXX
module cond(ifun,cc,Cnd);
    //设定功能的编码
    parameter C_YES = 4'h0;
    parameter C_LE = 4'h1;
    parameter C_L = 4'h2;
    parameter C_E = 4'h3;
    parameter C_NE = 4'h4;
    parameter C_GE = 4'h5;
    parameter C_G = 4'h6;
    input[3:0] ifun;
    input[2:0] cc;
    output Cnd;
    //将条件码分开
    wire zf = cc[2];
    wire sf = cc[1];
    wire of = cc[0];
    //计算Cnd
    assign Cnd = (ifun==C_YES)|
                 ifun==C_LE &((sf^of)|zf)|
                 (ifun == C_L & (sf^of)) |
                 ifun==C_E & zf|
                 ifun==C_NE & ~zf|
                 ifun==C_GE & (~sf^of)|
                 ifun==C_G & ((~sf^of)&~zf);
endmodule
//内存模块，用于储存数据和指令
//由16块内存芯片(bank)构成
//每次访存从部分芯片中取出一个字节
module myMerory(maddr,wenable,wdata,renable,rdata,m_ok,iaddr,instr,i_ok,clock);
    //内存总大小
    parameter memsize = 8192;
    input[63:0] maddr;
    input wenable;//Write enable
    input[63:0]wdata;
    input renable;
    output[63:0]rdata;
    output m_ok;
    input[63:0]iaddr;
    output[79:0]instr;
    output i_ok;
    input clock;
    //ibx对应于指令(instr)的第x低个字节
    wire[7:0] ib0,ib1,ib2,ib3,ib4,ib5,ib6,ib7,ib8,ib9;
    //dbx对应于数据输出(rdata)的第x低字节
    wire[7:0] db0,db1,db2,db3,db4,db5,db6,db7;
    wire[3:0]ibid = iaddr[3:0];//第一个bank的id，会影响其他bank的地址
    wire[59:0] iindex = iaddr[63:4];
    wire[59:0] iipl = iindex+1;//iindex的下一个地址
    wire[3:0] mbid = maddr[3:0];//数据地址的第一个bank
    wire[59:0] mindex = maddr[63:4];//bank内的数据地址
    wire[59:0] mipl = mindex+1;//mindex的下一个地址
    //声明指令地址，对应于不同的bank
    wire[59:0] addrI0,addrI1,addrI2,addrI3,addrI4,addrI5,addrI6,addrI7,addrI8,addrI9,addrI10,addrI11,addrI12,addrI13,addrI14,addrI15;
    //声明每个bank的指令输出
    wire[7:0] outI0,outI1,outI2,outI3,outI4,outI5,outI6,outI7,outI8,outI9,outI10,outI11,outI12,outI13,outI14,outI15;
    //每个bank的数据地址
    wire[59:0] addrD0,addrD1,addrD2,addrD3,addrD4,addrD5,addrD6,addrD7,addrD8,addrD9,addrD10,addrD11,addrD12,addrD13,addrD14,addrD15;
    //每个bank的数据输出
    wire[7:0]outD0,outD1,outD2,outD3,outD4,outD5,outD6,outD7,outD8,outD9,outD10,outD11,outD12,outD13,outD14,outD15;
    //每个bank的数据输入
    wire[7:0]inD0,inD1,inD2,inD3,inD4,inD5,inD6,inD7,inD8,inD9,inD10,inD11,inD12,inD13,inD14,inD15;
    //每个bank的写使能信号
    wire dwEn0,dwEn1,dwEn2,dwEn3,dwEn4,dwEn5,dwEn6,dwEn7,dwEn8,dwEn9,dwEn10,dwEn11,dwEn12,dwEn13,dwEn14,dwEn15;
    //声明16个内存芯片(bank)
    ram #(8,memsize/16,60) bank0(clock,addrI0,1'b0,8'b0,1'b1,outI0,addrD0,dwEn0,inD0,renable,outD0);
    ram #(8,memsize/16,60) bank1(clock,addrI1,1'b0,8'b0,1'b1,outI1,addrD1,dwEn1,inD1,renable,outD1);
    ram #(8,memsize/16,60) bank2(clock,addrI2,1'b0,8'b0,1'b1,outI2,addrD2,dwEn2,inD2,renable,outD2);
    ram #(8,memsize/16,60) bank3(clock,addrI3,1'b0,8'b0,1'b1,outI3,addrD3,dwEn3,inD3,renable,outD3);
    ram #(8,memsize/16,60) bank4(clock,addrI4,1'b0,8'b0,1'b1,outI4,addrD4,dwEn4,inD4,renable,outD4);
    ram #(8,memsize/16,60) bank5(clock,addrI5,1'b0,8'b0,1'b1,outI5,addrD5,dwEn5,inD5,renable,outD5);
    ram #(8,memsize/16,60) bank6(clock,addrI6,1'b0,8'b0,1'b1,outI6,addrD6,dwEn6,inD6,renable,outD6);
    ram #(8,memsize/16,60) bank7(clock,addrI7,1'b0,8'b0,1'b1,outI7,addrD7,dwEn7,inD7,renable,outD7);
    ram #(8,memsize/16,60) bank8(clock,addrI8,1'b0,8'b0,1'b1,outI8,addrD8,dwEn8,inD8,renable,outD8);
    ram #(8,memsize/16,60) bank9(clock,addrI9,1'b0,8'b0,1'b1,outI9,addrD9,dwEn9,inD9,renable,outD9);
    ram #(8,memsize/16,60) bank10(clock,addrI10,1'b0,8'b0,1'b1,outI10,addrD10,dwEn10,inD10,renable,outD10);
    ram #(8,memsize/16,60) bank11(clock,addrI11,1'b0,8'b0,1'b1,outI11,addrD11,dwEn11,inD11,renable,outD11);
    ram #(8,memsize/16,60) bank12(clock,addrI12,1'b0,8'b0,1'b1,outI12,addrD12,dwEn12,inD12,renable,outD12);
    ram #(8,memsize/16,60) bank13(clock,addrI13,1'b0,8'b0,1'b1,outI13,addrD13,dwEn13,inD13,renable,outD13);
    ram #(8,memsize/16,60) bank14(clock,addrI14,1'b0,8'b0,1'b1,outI14,addrD14,dwEn14,inD14,renable,outD14);
    ram #(8,memsize/16,60) bank15(clock,addrI15,1'b0,8'b0,1'b1,outI15,addrD15,dwEn15,inD15,renable,outD15);
    //计算出每个bank访存地址(每个bank取出一个字节，组合在一起）
    assign addrI0 = ibid>=7 ? iipl:iindex;
    assign addrI1 = ibid>=8 ? iipl:iindex;
    assign addrI2 = ibid>=9 ? iipl:iindex;
    assign addrI3 = ibid>=10 ? iipl:iindex;
    assign addrI4 = ibid>=11 ? iipl:iindex;
    assign addrI5 = ibid>=12 ? iipl:iindex;
    assign addrI6 = ibid>=13 ? iipl:iindex;
    assign addrI7 = ibid>=14 ? iipl:iindex;
    assign addrI8 = ibid>=15 ? iipl:iindex;
    assign addrI9 = iindex;
    assign addrI10 = iindex;
    assign addrI11 = iindex;
    assign addrI12 = iindex;
    assign addrI13 = iindex;
    assign addrI14 = iindex;
    assign addrI15 = iindex;
    //判断指令访问是否越界
    assign i_ok = (iaddr+9)<memsize;
    //计算指令每个字节对应哪个bank中取出的字节
    assign	ib0 = !i_ok ? 0 :
        ibid == 0 ? outI0 :
        ibid == 1 ? outI1 :
        ibid == 2 ? outI2 :
        ibid == 3 ? outI3 :
        ibid == 4 ? outI4 :
        ibid == 5 ? outI5 :
        ibid == 6 ? outI6 :
        ibid == 7 ? outI7 :
        ibid == 8 ? outI8 :
        ibid == 9 ? outI9 :
        ibid == 10 ? outI10 :
        ibid == 11 ? outI11 :
        ibid == 12 ? outI12 :
        ibid == 13 ? outI13 :
        ibid == 14 ? outI14 :
        outI15;  
    assign	ib1 = !i_ok ? 0 :
            ibid == 0 ? outI1 :
            ibid == 1 ? outI2 :
            ibid == 2 ? outI3 :
            ibid == 3 ? outI4 :
            ibid == 4 ? outI5 :
            ibid == 5 ? outI6 :
            ibid == 6 ? outI7 :
            ibid == 7 ? outI8 :
            ibid == 8 ? outI9 :
            ibid == 9 ? outI10 :
            ibid == 10 ? outI11 :
            ibid == 11 ? outI12 :
            ibid == 12 ? outI13 :
            ibid == 13 ? outI14 :
            ibid == 14 ? outI15 :
            outI0;    
        assign	ib2 = !i_ok ? 0 :
                    ibid == 0 ? outI2 :
                    ibid == 1 ? outI3 :
                    ibid == 2 ? outI4 :
                    ibid == 3 ? outI5 :
                    ibid == 4 ? outI6 :
                    ibid == 5 ? outI7 :
                    ibid == 6 ? outI8 :
                    ibid == 7 ? outI9 :
                    ibid == 8 ? outI10 :
                    ibid == 9 ? outI11 :
                    ibid == 10 ? outI12 :
                    ibid == 11 ? outI13 :
                    ibid == 12 ? outI14 :
                    ibid == 13 ? outI15 :
                    ibid == 14 ? outI1 :
                    outI0;   
    assign	ib3 = !i_ok ? 0 :
                        ibid == 0 ? outI3 :
                        ibid == 1 ? outI4 :
                        ibid == 2 ? outI5 :
                        ibid == 3 ? outI6 :
                        ibid == 4 ? outI7 :
                        ibid == 5 ? outI8 :
                        ibid == 6 ? outI9 :
                        ibid == 7 ? outI10 :
                        ibid == 8 ? outI11 :
                        ibid == 9 ? outI12 :
                        ibid == 10 ? outI13 :
                        ibid == 11 ? outI14 :
                        ibid == 12 ? outI15 :
                        ibid == 13 ? outI0 :
                        ibid == 14 ? outI1 :
                        outI2;      
    assign	ib4 = !i_ok ? 0 :
                            ibid == 0    ? outI4 :
                            ibid == 1    ? outI5 :
                            ibid == 2    ? outI6 :
                            ibid == 3    ? outI7 :
                            ibid == 4    ? outI8 :
                            ibid == 5    ? outI9 :
                            ibid == 6    ? outI10 :
                            ibid == 7    ? outI11 :
                            ibid == 8    ? outI12 :
                            ibid == 9    ? outI13 :
                            ibid == 10    ? outI14 :
                            ibid == 11    ? outI15 :
                            ibid == 12    ? outI0 :
                            ibid == 13    ? outI1 :
                            ibid == 14    ? outI2 :
                            outI3;    
    assign	ib5 = !i_ok ? 0 :
                                ibid == 0    ? outI5 :
                                ibid == 1    ? outI6 :
                                ibid == 2    ? outI7 :
                                ibid == 3    ? outI8 :
                                ibid == 4    ? outI9 :
                                ibid == 5    ? outI10 :
                                ibid == 6    ? outI11 :
                                ibid == 7    ? outI12 :
                                ibid == 8    ? outI13 :
                                ibid == 9    ? outI14 :
                                ibid == 10    ? outI15 :
                                ibid == 11    ? outI0 :
                                ibid == 12    ? outI1 :
                                ibid == 13    ? outI2 :
                                ibid == 14	? outI3 :
                                outI4;    
    assign	ib6 = !i_ok ? 0 :
                                    ibid == 0    ? outI6 :
                                    ibid == 1    ? outI7 :
                                    ibid == 2    ? outI8 :
                                    ibid == 3    ? outI9 :
                                    ibid == 4    ? outI10 :
                                    ibid == 5    ? outI11 :
                                    ibid == 6    ? outI12 :
                                    ibid == 7    ? outI13 :
                                    ibid == 8    ? outI14 :
                                    ibid == 9    ? outI15 :
                                    ibid == 10    ? outI0 :
                                    ibid == 11    ? outI1 :
                                    ibid == 12    ? outI2 :
                                    ibid == 13    ? outI3 :
                                    ibid == 14    ? outI4 :
                                    outI5;    
    assign	ib7 = !i_ok ? 0 :
                                        ibid == 0    ? outI7 :
                                        ibid == 1    ? outI8 :
                                        ibid == 2    ? outI9 :
                                        ibid == 3    ? outI10 :
                                        ibid == 4    ? outI11 :
                                        ibid == 5    ? outI12 :
                                        ibid == 6    ? outI13 :
                                        ibid == 7    ? outI14 :
                                        ibid == 8    ? outI15 :
                                        ibid == 9    ? outI0 :
                                        ibid == 10    ? outI1 :
                                        ibid == 11    ? outI2 :
                                        ibid == 12    ? outI3 :
                                        ibid == 13    ? outI4 :
                                        ibid == 14    ? outI5 :
                                        outI6;    
    assign	ib8 = !i_ok ? 0 :
                                            ibid == 0    ? outI8 :
                                            ibid == 1    ? outI9 :
                                            ibid == 2    ? outI10 :
                                            ibid == 3    ? outI11 :
                                            ibid == 4    ? outI12 :
                                            ibid == 5    ? outI13 :
                                            ibid == 6    ? outI14 :
                                            ibid == 7    ? outI15 :
                                            ibid == 8    ? outI0 :
                                            ibid == 9    ? outI1 :
                                            ibid == 10    ? outI2 :
                                            ibid == 11    ? outI3 :
                                            ibid == 12    ? outI4 :
                                            ibid == 13	?	outI5 :
                                            ibid == 14    ? outI6 :
                                            outI7;        
    assign	ib9 = !i_ok ? 0 :
                                                ibid == 0    ? outI9 :
                                                ibid == 1    ? outI10 :
                                                ibid == 2    ? outI11 :
                                                ibid == 3    ? outI12 :
                                                ibid == 4    ? outI13 :
                                                ibid == 5    ? outI14 :
                                                ibid == 6    ? outI15 :
                                                ibid == 7    ? outI0 :
                                                ibid == 8    ? outI1 :
                                                ibid == 9    ? outI2 :
                                                ibid == 10    ? outI3 :
                                                ibid == 11    ? outI4 :
                                                ibid == 12    ? outI5 :
                                                ibid == 13    ? outI6 :
                                                ibid == 14    ? outI7 :
                                                outI8; 
    //将取出的10个字节组合成指令，icode和ifun在最低一个字节
    assign	  instr[7:0] = ib0;
    assign    instr[15:8] = ib1;
    assign    instr[23:16] = ib2;
    assign    instr[31:24] = ib3;
    assign    instr[39:32] = ib4;
    assign    instr[47:40] = ib5;
    assign    instr[55:48] = ib6;
    assign    instr[63:56] = ib7;
    assign    instr[71:64] = ib8;
    assign    instr[79:72] = ib9;    
    //判断访存是否出错  
    assign m_ok = (!renable & !wenable)|(maddr+7)<memsize;
    //计算每个bank所取出字节的地址
    assign	addrD0 = mbid >=  9 ? mipl : mindex;
    assign    addrD1 = mbid >= 10 ? mipl : mindex;
    assign    addrD2 = mbid >= 11 ? mipl : mindex;
    assign    addrD3 = mbid >= 12 ? mipl : mindex;
    assign    addrD4 = mbid >= 13 ? mipl : mindex;
    assign    addrD5 = mbid >= 14 ? mipl : mindex;
    assign    addrD6 = mbid >= 15 ? mipl : mindex;
    assign    addrD7 = mindex;
    assign    addrD8 = mindex;
    assign    addrD9 = mindex;
    assign    addrD10 =    mindex;
    assign    addrD11 =    mindex;
    assign    addrD12 =    mindex;
    assign    addrD13 =    mindex;
    assign    addrD14 =    mindex;
    assign    addrD15 =    mindex;  
    //计算数据输出的各个字节   
    assign	db0 = !m_ok ? 0	:
        mbid == 0    ? outD0    :
        mbid == 1    ? outD1    :
        mbid == 2    ? outD2    :
        mbid == 3    ? outD3    :
        mbid == 4    ? outD4    :
        mbid == 5    ? outD5    :
        mbid == 6    ? outD6    :
        mbid == 7    ? outD7    :
        mbid == 8    ? outD8    :
        mbid == 9    ? outD9    :
        mbid == 10    ? outD10 :
        mbid == 11    ? outD11 :
        mbid == 12    ? outD12 :
        mbid == 13    ? outD13 :
        mbid == 14    ? outD14    :
        outD15;        
    assign	db1 = !m_ok ? 0	:
            mbid == 0    ? outD1    :
            mbid == 1    ? outD2    :
            mbid == 2    ? outD3    :
            mbid == 3    ? outD4    :
            mbid == 4    ? outD5    :
            mbid == 5    ? outD6    :
            mbid == 6    ? outD7    :
            mbid == 7    ? outD8    :
            mbid == 8    ? outD9    :
            mbid == 9    ? outD10 :
            mbid == 10    ? outD11 :
            mbid == 11    ? outD12 :
            mbid == 12    ? outD13 :
            mbid == 13    ? outD14 :
            mbid == 14    ? outD15    :
            outD0;
    assign	db2 = !m_ok ? 0	:
                mbid == 0    ? outD2    :
                mbid == 1    ? outD3    :
                mbid == 2    ? outD4    :
                mbid == 3    ? outD5    :
                mbid == 4    ? outD6    :
                mbid == 5    ? outD7    :
                mbid == 6    ? outD8    :
                mbid == 7    ? outD9    :
                mbid == 8    ? outD10 :
                mbid == 9    ? outD11 :
                mbid == 10    ? outD12 :
                mbid == 11    ? outD13 :
                mbid == 12	? outD14 :
                mbid == 13    ? outD15    :
                mbid == 14    ? outD0    :
                outD1;        
    assign	db3 = !m_ok ? 0	:
                    mbid == 0    ? outD3    :
                    mbid == 1    ? outD4    :
                    mbid == 2    ? outD5    :
                    mbid == 3    ? outD6    :
                    mbid == 4    ? outD7    :
                    mbid == 5    ? outD8    :
                    mbid == 6    ? outD9    :
                    mbid == 7    ? outD10 :
                    mbid == 8    ? outD11 :
                    mbid == 9    ? outD12 :
                    mbid == 10    ? outD13 :
                    mbid == 11    ? outD14 :
                    mbid == 12    ? outD15    :
                    mbid == 13    ? outD0    :
                    mbid == 14    ? outD1    :
                    outD2;        
    assign	db4 = !m_ok ? 0	:
                        mbid == 0    ? outD4    :
                        mbid == 1    ? outD5    :
                        mbid == 2    ? outD6    :
                        mbid == 3    ? outD7    :
                        mbid == 4    ? outD8    :
                        mbid == 5    ? outD9    :
                        mbid == 6    ? outD10 :
                        mbid == 7    ? outD11 :
                        mbid == 8    ? outD12 :
                        mbid == 9    ? outD13 :
                        mbid == 10    ? outD14 :
                        mbid == 11    ? outD15    :
                        mbid == 12    ? outD0    :
                        mbid == 13    ? outD1    :
                        mbid == 14    ? outD2    :
                        outD3;        
    assign	db5 = !m_ok ? 0 :	
                            mbid == 0    ? outD5    :
                            mbid == 1    ? outD6    :
                            mbid == 2    ? outD7    :
                            mbid == 3    ? outD8    :
                            mbid == 4    ? outD9    :
                            mbid == 5    ? outD10 :
                            mbid == 6    ? outD11 :
                            mbid == 7    ? outD12 :
                            mbid == 8    ? outD13 :
                            mbid == 9    ? outD14 :
                            mbid == 10    ? outD15 :
                            mbid == 11	?	outD0	:
                            mbid == 12    ?    outD1    :
                            mbid == 13    ?    outD2    :
                            mbid == 14    ? outD3    :
                            outD4;            
    assign	db6 = !m_ok ? 0	:
                                mbid == 0    ? outD6    :
                                mbid == 1    ? outD7    :
                                mbid == 2    ? outD8    :
                                mbid == 3    ? outD9    :
                                mbid == 4    ? outD10 :
                                mbid == 5    ? outD11 :
                                mbid == 6    ? outD12 :
                                mbid == 7    ? outD13 :
                                mbid == 8    ? outD14 :
                                mbid == 9    ? outD15    :
                                mbid == 10    ? outD0    :
                                mbid == 11    ? outD1    :
                                mbid == 12    ? outD2    :
                                mbid == 13    ? outD3    :
                                mbid == 14    ? outD4    :
                                outD5;            
    assign	db7 = !m_ok ? 0	:
                                    mbid == 0    ? outD7    :
                                    mbid == 1    ? outD8    :
                                    mbid == 2    ? outD9    :
                                    mbid == 3    ? outD10 :
                                    mbid == 4    ? outD11 :
                                    mbid == 5    ? outD12 :
                                    mbid == 6    ? outD13 :
                                    mbid == 7    ? outD14 :
                                    mbid == 8    ? outD15    :
                                    mbid == 9    ? outD0    :
                                    mbid == 10    ? outD1    :
                                    mbid == 11    ? outD2    :
                                    mbid == 12    ? outD3    :
                                    mbid == 13    ? outD4    :
                                    mbid == 14    ? outD5    :
                                    outD6;       
    //将数据输出合并     
    assign	rdata[ 7: 0] = db0;
    assign    rdata[15: 8] = db1;
    assign    rdata[23:16] = db2;
    assign    rdata[31:24] = db3;
    assign    rdata[39:32] = db4;
    assign    rdata[47:40] = db5;
    assign    rdata[55:48] = db6;
    assign    rdata[63:56] = db7; 
    //将数据输出按字节拆分
    wire [7:0]wd0 = wdata[ 7: 0];
    wire [7:0]wd1 = wdata[ 15: 8];
    wire [7:0]wd2 = wdata[ 23: 16];
    wire [7:0]wd3 = wdata[ 31: 24];    
    wire [7:0]wd4 = wdata[39:32];
    wire [7:0]wd5 = wdata[47:40];
    wire [7:0]wd6 = wdata[55:48]; 
    wire [7:0]wd7 = wdata[63:56];
    //判断每个字节要写入哪个bank
    assign	inD0 =		
        mbid == 9    ? wd7 :
        mbid == 10 ? wd6 :
        mbid == 11 ? wd5 :
        mbid == 12 ? wd4 :
        mbid == 13 ? wd3 :
        mbid == 14 ? wd2 :
        mbid == 15 ? wd1 :
        mbid == 0 ? wd0 :
        0;        
    assign	inD1 =	
            mbid == 10 ? wd7 :
            mbid == 11 ? wd6 :
            mbid == 12 ? wd5 :
            mbid == 13 ? wd4 :
            mbid == 14 ? wd3 :
            mbid == 15    ? wd2 :
            mbid == 0    ? wd1 :
            mbid == 1    ? wd0 :
            0;    
    assign	inD2 =		
                mbid == 11 ? wd7 :
                mbid == 12 ? wd6 :
                mbid == 13 ? wd5 :
                mbid == 14 ? wd4 :
                mbid == 15    ? wd3 :
                mbid == 0    ? wd2 :
                mbid == 1    ? wd1 :
                mbid == 2    ? wd0 :
                0;        
    assign	inD3 =		
                    mbid == 12 ? wd7 :
                    mbid == 13 ? wd6 :
                    mbid == 14 ? wd5 :
                    mbid == 15 ? wd4 :
                    mbid == 0 ? wd3    :
                    mbid == 1 ? wd2    :
                    mbid == 2 ? wd1    :
                    mbid == 3 ? wd0    :
                    0;
    assign	inD4 =	
                        mbid == 13 ? wd7 :
                        mbid == 14 ? wd6 :
                        mbid == 15 ? wd5 :
                        mbid == 0 ? wd4    :
                        mbid == 1 ? wd3    :
                        mbid == 2 ? wd2    :
                        mbid == 3 ? wd1    :
                        mbid == 4 ? wd0    :
                        0;    
    assign	inD5 =	
                            mbid == 14 ? wd7 :
                            mbid == 15 ? wd6 :
                            mbid == 0 ? wd5    :
                            mbid == 1 ? wd4    :
                            mbid == 2 ? wd3    :
                            mbid == 3 ? wd2    :
                            mbid == 4 ? wd1    :
                            mbid == 5 ? wd0    :
                            0;    
    assign	inD6 =	
                                mbid == 15 ? wd7 :
                                mbid == 0 ? wd6    :
                                mbid == 1 ? wd5    :
                                mbid == 2 ? wd4    :
                                mbid == 3 ? wd3    :
                                mbid == 4 ? wd2    :
                                mbid == 5 ? wd1    :
                                mbid == 6 ? wd0    :
                                0;    
    assign	inD7 =	
                                    mbid == 0 ? wd7    :
                                    mbid == 1 ? wd6    :
                                    mbid == 2 ? wd5    :
                                    mbid == 3 ? wd4    :
                                    mbid == 4 ? wd3    :
                                    mbid == 5 ? wd2    :
                                    mbid == 6 ? wd1    :
                                    mbid == 7 ? wd0    :
                                    0;    
    assign	inD8 =	
                                        mbid == 1 ? wd7    :
                                        mbid == 2 ? wd6    :
                                        mbid == 3 ? wd5    :  
                                        mbid == 4 ? wd4	:
                                        mbid == 5 ? wd3    :
                                        mbid == 6 ? wd2    :
                                        mbid == 7 ? wd1    :
                                        mbid == 8 ? wd0    :
                                        0;    
    assign	inD9 =	
                                            mbid == 2 ? wd7    :
                                            mbid == 3 ? wd6    :
                                            mbid == 4 ? wd5    :
                                            mbid == 5 ? wd4    :
                                            mbid == 6 ? wd3    :
                                            mbid == 7 ? wd2    :
                                            mbid == 8 ? wd1    :
                                            mbid == 9 ? wd0    :
                                            0;    
    assign	inD10 =	
                                                mbid == 3 ? wd7    :
                                                mbid == 4 ? wd6    :
                                                mbid == 5 ? wd5    :
                                                mbid == 6 ? wd4    :
                                                mbid == 7 ? wd3    :
                                                mbid == 8 ? wd2    :
                                                mbid == 9 ? wd1    :
                                                mbid == 10 ? wd0 :
                                                0;    
    assign	inD11 =	
                                                    mbid == 4 ? wd7    :
                                                    mbid == 5 ? wd6    :
                                                    mbid == 6 ? wd5    :
                                                    mbid == 7 ? wd4    :
                                                    mbid == 8 ? wd3    :
                                                    mbid == 9 ? wd2    :
                                                    mbid == 10 ? wd1 :
                                                    mbid == 11 ? wd0 :
                                                    0;    
    assign	inD12 =	
                                                        mbid == 5 ? wd7    :
                                                        mbid == 6 ? wd6    :
                                                        mbid == 7 ? wd5    :
                                                        mbid == 8 ? wd4    :
                                                        mbid == 9 ? wd3    :
                                                        mbid == 10 ? wd2 :
                                                        mbid == 11 ? wd1 :
                                                        mbid == 12 ? wd0 :
                                                        0;   
    assign	inD13 =	
                                                            mbid == 6 ? wd7    :
                                                            mbid == 7 ? wd6    :
                                                            mbid == 8 ? wd5    :
                                                            mbid == 9 ? wd4    :
                                                            mbid == 10 ? wd3    :
                                                            mbid == 11 ? wd2    :
                                                            mbid == 12 ? wd1    :
                                                            mbid == 13 ? wd0 :
                                                            0;    
    assign	inD14 =	
    mbid == 7 ? wd7    :
    mbid == 8 ? wd6    :
    mbid == 9 ? wd5    :
    mbid == 10 ? wd4    :
    mbid == 11 ? wd3    :
    mbid == 12 ? wd2    :
    mbid == 13 ? wd1    :
    mbid == 14 ? wd0 :
    0;    
    assign	inD15 =	
        mbid == 8 ? wd7    :
        mbid == 9 ? wd6    :
        mbid == 10 ? wd5    :
        mbid == 11 ? wd4    :
        mbid == 12 ? wd3    :
        mbid == 13 ? wd2    :
        mbid == 14 ? wd1    :
        mbid == 15 ? wd0 :
        0;    
        //计算16个bank的写入使能
        assign	dwEn0 = wenable & (mbid <= 0 | mbid >= 9);
        assign    dwEn1 = wenable & (mbid <= 1 | mbid >= 10);
        assign    dwEn2 = wenable & (mbid <= 2 | mbid >= 11);
        assign    dwEn3 = wenable & (mbid <= 3 | mbid >= 12);
        assign    dwEn4 = wenable & (mbid <= 4 | mbid >= 13);
        assign    dwEn5 = wenable & (mbid <= 5 | mbid >= 14);
        assign    dwEn6 = wenable & (mbid <= 6 | mbid >= 15);
        assign    dwEn7 = wenable & (mbid <= 7);
        assign    dwEn8 = wenable & (mbid >= 1 & mbid <= 8);
        assign    dwEn9 = wenable & (mbid >= 2 & mbid <= 9);
        assign    dwEn10 = wenable & (mbid >= 3 & mbid <= 10);
        assign    dwEn11 = wenable & (mbid >= 4 & mbid <= 11);
        assign    dwEn12 = wenable & (mbid >= 5 & mbid <= 12);
        assign    dwEn13 = wenable & (mbid >= 6 & mbid <= 13);
        assign    dwEn14 = wenable & (mbid >= 7 & mbid <= 14);
        assign	  dwEn15 = wenable & (mbid >= 8); 
endmodule
//seq流水线    
//mode有5种模式
// RUN: Normal operation
//RESET: Sets PC to 0,clears all regeisters;Initializes condtion codes
//DOWNLOAD:Download bytes from controller into memory
//UPLOAD:Upload bytes form memory to controller
//STATUS:Upload other status information to controller  
//当进入DOWNLOAD或者UPLOAD模式的时候udaddr指明了访存地址
//  idata和odata为要输入或者输出的信息
//stat为seq的状态
//clock为同步时钟                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
module seq(mode, stat, odata, udaddr, idata, clock);
output[63:0]odata;
output[1:0]stat;
input clock;
input[2:0]mode;
input[63:0]udaddr, idata;
    //声明模式的编码
    parameter RUN_MODE = 0;
    parameter RESET_MODE = 1;
    parameter DOWNLOAD_MODE = 2;
    parameter UPLOAD_MODE = 3;
    parameter STATUS_MODE = 4;
    //声明指令的编码
    parameter IHALT = 4'h0;
    parameter INOP = 4'h1;
    parameter IRRMOVQ = 4'h2;
    parameter IIRMOVQ = 4'h3;
    parameter IRMMOVQ = 4'h4;
    parameter IMRMOVQ = 4'h5;
    parameter IOPQ = 4'h6;
    parameter IJXX = 4'h7;
    parameter ICALL = 4'h8;
    parameter IRET = 4'h9;
    parameter IPUSHQ = 4'hA;
    parameter IPOPQ = 4'hB;
    parameter IIADDQ = 4'hC;
    
    parameter FNONE = 4'h0;
    parameter UNCOND = 4'h0;
    parameter RRSP = 4'h4;
    parameter RRBP = 4'h5;
    parameter RNONE = 4'hF;
    //声明ALUADD的编码
    parameter ALUADD = 4'h0;
    //声明stat各种状态编码
    parameter SAOK = 2'h0;
    parameter SHLT = 2'h1;
    parameter SADR = 2'h2;
    parameter SINS = 2'h3;
    //取指阶段
    wire[63:0] pc,f_valM,f_valC,f_valP;
    wire imem_error,instr_valid,need_valC,need_regids,f_ok,f_Cnd;
    wire[3:0]icode,ifun,imem_icode,imem_ifun,rA,rB,f_Icode;
    wire[63:0]valC,valP;
    wire[79:0]instr;
   //译码阶段
   wire[3:0]srcA,srcB,dstE,dstM;
   wire[63:0]valA,valB;
   //执行阶段
   wire set_cc,Cnd;
   wire[2:0]cc,new_cc;
   wire[3:0]alufun;
   wire[63:0]aluA,aluB,valE;
   //访存阶段
   wire[63:0]memAddr,memData,valM;
   wire memRead,memWrite,dmem_error,m_ok;
   //写回阶段没有额外的变量
   wire[1:0]Stat;
   wire[63:0]rax,rcx,rdx,rbx,rsp,rbp,rdi,rsi,r8,r9,r10,r11,r12,r13,r14;
   wire zf = cc[2];
   wire sf = cc[1];
   wire of = cc[0];
   //mode
   wire resetting = (mode== RESET_MODE);
   wire uploading = (mode== UPLOAD_MODE);
   wire downloading = (mode==DOWNLOAD_MODE);
   wire running = (mode == RUN_MODE);
   wire getting_info = (mode == STATUS_MODE);
   
   assign stat = Stat;
   assign odata = getting_info ? (udaddr==0 ? rax:
                                  udaddr==8 ? rcx:
                                  udaddr==16 ? rdx:
                                  udaddr==24 ? rbx:
                                  udaddr==32 ? rsp:
                                  udaddr==40 ? rbp:
                                  udaddr==48 ? rsi:
                                  udaddr==56 ? rdi:
                                  udaddr==64 ? r8:
                                  udaddr==72 ? r9:
                                  udaddr==80 ? r10:
                                  udaddr==88 ? r11:
                                  udaddr==96 ? r12:
                                  udaddr==104 ? r13:
                                  udaddr==112 ? r14:
                                  udaddr==120 ? cc:0):valM;
// 寄存器部分
//取指逻辑
cenrreg #(4)pIcode(icode,f_Icode,running,resetting,4'b0,clock);
cenrreg #(1)pCnd(Cnd,f_Cnd,running,resetting,1'b0,clock);
cenrreg #(64)pValM(valM,f_valM,running,resetting,64'h0,clock);
cenrreg #(64)pValC(valC,f_valC,running,resetting,64'h0,clock);
cenrreg #(64)pValP(valP,f_valP,running,resetting,64'h0,clock);
split split(instr[7:0],imem_icode,imem_ifun);
align align(instr[79:8],rA,rB,valC,need_regids);
pc_increment pci(pc,need_regids,need_valC,valP);
//译码阶段
registerFile regf(srcA,srcB,valA,valB,dstE,valE,dstM,valM,resetting,clock,rax,rcx,rdx,rbx,rsp,rbp,rdi,rsi,r8,r9,r10,r11,r12,r13,r14);
//执行阶段
alu alu(aluA,aluB,ifun,valE,new_cc);
CC ccreg(new_cc,cc,running&set_cc,resetting,clock);
cond cond_check(ifun,cc,Cnd);
//访存阶段
myMerory m(
    (uploading|downloading)?udaddr:memAddr,running&memWrite|downloading,downloading ? idata:memData,running&memRead|uploading,valM,m_ok,
    pc,instr,f_ok,clock);
//写回阶段

//控制逻辑
//取指阶段
assign pc = ((f_Icode==IJXX)&f_Cnd)|(f_Icode==ICALL) ? f_valC: (f_Icode==IRET) ? f_valM : f_valP;
assign imem_error = ~f_ok;
assign ifun = imem_error ? FNONE : 
              imem_ifun;
assign icode = imem_error ? INOP: 
               imem_icode;
assign instr_valid = (icode ==INOP)|(icode==IHALT)|(icode==IRRMOVQ)|(icode==IIRMOVQ)|(icode==IRMMOVQ)|(icode==IMRMOVQ)|
	       (icode==IOPQ)|(icode==IJXX)|(icode==ICALL)|(icode==IRET)|(icode==IPUSHQ)|(icode==IPOPQ)|(icode==IIADDQ);
assign need_valC = (icode == IIRMOVQ|icode == IRMMOVQ|icode == IMRMOVQ|icode == IJXX|icode == ICALL|icode == IIADDQ);
assign need_regids = (icode==IRRMOVQ|icode==IOPQ|icode==IPUSHQ|icode==IPOPQ|icode==IIRMOVQ|icode==IRMMOVQ|icode==IMRMOVQ|icode==IIADDQ);
//译码阶段
assign srcA =(icode==IRRMOVQ|icode==IRMMOVQ|icode==IOPQ|icode==IPUSHQ) ?  rA : 
             (icode==IPOPQ|icode==IRET) ? RRSP:
             RNONE;
assign srcB = (icode==IOPQ|icode==IRMMOVQ|icode==IMRMOVQ|icode==IIADDQ) ? rB:
              (icode==IPUSHQ|icode==IPOPQ|icode==ICALL|icode==IRET) ? RRSP:
               RNONE;
assign dstE = (icode==IRRMOVQ|icode==IIRMOVQ|icode==IOPQ|icode==IIADDQ) ? rB:
              (icode==IPUSHQ|icode==IPOPQ|icode==ICALL|icode==IRET) ? RRSP:             
              RNONE;
assign dstM =  (icode==IMRMOVQ|icode==IPOPQ) ? rA:
               RNONE;
//执行阶段
assign aluA =  (icode==IRRMOVQ|icode==IOPQ) ? valA:
               (icode==IIRMOVQ|icode==IRMMOVQ|icode==IMRMOVQ|icode==IIADDQ) ? valC:
               (icode==ICALL|icode==IPUSHQ) ? -8:
               (icode==IRET|icode==IPOPQ) ? 8 :
               0;
assign aluB = (icode==IRMMOVQ|icode==IMRMOVQ|icode==IOPQ|icode==ICALL|icode==IPUSHQ|icode==IRET|icode==IPOPQ|icode==IIADDQ) ? valB:
              (icode==IRRMOVQ|icode==IIRMOVQ) ? 0:
              0;
assign alufun = (icode == IOPQ) ? ifun:
                 ALUADD;
assign set_cc = (icode==IOPQ|icode==IIADDQ);
//访存阶段
assign dmem_error = ~m_ok;
assign memRead =  (icode==IMRMOVQ|icode==IPOPQ|icode==IRET);  
assign memWrite = (icode==IRMMOVQ|icode==IPUSHQ|icode==ICALL);
assign  memAddr = (icode==IRMMOVQ|icode==IPUSHQ|icode==ICALL|icode==IMRMOVQ) ?  valE:
                  (icode==IPOPQ|icode==IRET) ? valA:
                  0;
assign memData = (icode==IRMMOVQ|icode==IPUSHQ) ? valA:
                 icode==ICALL ? valP :
                 0;
assign Stat =  (imem_error || dmem_error) ?  SADR:
                     (!instr_valid) ? SINS:
                     (icode == IHALT) ? SHLT:
                      SAOK;
endmodule
module seq_synthesis(mode,stat,clock,addr,odata);
input[2:0]mode;
output[1:0]stat;
input clock;
input[1:0]addr;
output[7:0]odata;
wire[63:0] idata;
wire[63:0] udaddr;
wire[63:0]data;
assign odata = data[7:0];
seq mySeq(mode,stat,data,udaddr,idata,clock);
assign udaddr = addr == 2'b00 ? 64'b0000_0000_0000_0000:
                        addr == 2'b01 ? 64'b0000_0000_0000_1000:
                        addr == 2'b10 ? 64'h0000_0000_0001_0000:64'h0;
assign idata = addr== 2'b00 ? 64'h0000_0000_0003_f0c0:
               addr== 2'b01 ? 64'h0000_0000_0120_0000:
               addr== 2'b10 ? 64'h0000_0000_0000_0000:64'h0000_0000_0003_f0c0;
endmodule