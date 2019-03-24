module processor();
reg [6:0]address;
wire [31:0] instruction, instructionO, Offset_Extended, aluReg2, read_data_1, read_data_2, write_data,
            ReadReg1O, ReadReg2O, OffsetO, out, AluOut, readData,AluResO,WriteDataO,MemoryWordReadO;													//labeling wire
wire [15:0] Offset;
wire [6:0] PC, PCO, incAddress, incAddress0, BranchAddress, BranchAddressO;
wire [5:0] funct;
wire [4:0] write_reg, Shamt, Rs, Rt, Rd, EX, read_reg_1, read_reg_2,
           ShamtO, RtO, RdO,RdOO,DESTINATION;
wire [2:0] MEM, MEMO, ALUop, ALUcontrol;
wire [1:0] WB, WBO, WBOO;
wire  zeroFlag, zeroFlagO,MemWrite,MemRead,Branch;
wire RegDst;
wire ALUSrc;
wire MemToReg;
reg PCsrc;
wire regWrite;
reg clk;

assign incAddress=address+4;                       // PC+4

initial             //clock and initial values
begin
PCsrc=0;
clk = 0;
address=0;
forever
	begin
		#10 clk = ~clk;
	end
end

initial               //finish
begin
    #200 $finish;
end

//PC MUX ()
always @(posedge clk)
begin
	address<= (PCsrc==1)? BranchAddressO: incAddress;
end

//IF block
instructionMemory IM(instruction, address);
IF_ID IFID(instructionO,incAddress0,instruction,incAddress,clk);

//id block
assign Shamt = {instructionO[10],instructionO[9],instructionO[8],instructionO[7],instructionO[6]};
assign Offset = {instructionO[15],instructionO[14],instructionO[13],instructionO[12],instructionO[11],
            instructionO[10],instructionO[9],instructionO[8],instructionO[7],instructionO[6],
            instructionO[5],instructionO[4],instructionO[3],instructionO[2],instructionO[1],instructionO[0]};
assign Rs = {instructionO[25],instructionO[24],instructionO[23],instructionO[22],instructionO[21]};
assign Rt = {instructionO[20],instructionO[19],instructionO[18],instructionO[17],instructionO[16]};
assign Rd = {instructionO[15],instructionO[14],instructionO[13],instructionO[12],instructionO[11]};
assign Offset_Extended = (Offset[15]==1)? {16'hffff,Offset}: {16'h0000,Offset};
assign funct = {OffsetO[5],OffsetO[4],OffsetO[3],OffsetO[2],OffsetO[1],OffsetO[0]};
registerFile RF(Rs, Rt, write_reg, write_data, regWrite, read_data_1, read_data_2, clk);
controlUnit CUcontrolUnit(instructionO, EX, MEM, WB,Shamt,bne);
ID_EX IDEX(PCO,ReadReg1O,ReadReg2O,ShamtO,OffsetO,RtO,RdO,WBO,MEMO,ALUop,ALUSrc,RegDst,
        incAddress0,bneO,clk,read_data_1,read_data_2,Shamt,Offset_Extended,Rt,Rd,WB,MEM,EX,bne);
		
//EX block
assign aluReg2 = (ALUSrc==1)? OffsetO:ReadReg2O;
assign DESTINATION = (RegDst==1)? RdO:RtO;
assign BranchAddress= PCO+OffsetO*4;
ALU alu(zeroFlag,out,ReadReg1O,aluReg2,ALUcontrol,ShamtO);
ALUControl aluc(ALUcontrol,ALUop,funct);
EX_MEM exMem(WriteDataO,AluOut,BranchAddressO,RdOO,WBOO,MemRead, MemWrite, Branch,zeroFlagO,bneOO,clk,
            ReadReg2O,out,BranchAddress,DESTINATION,WBO,MEMO,zeroFlag,bneO);
			
//mem block
always@(zeroFlagO,Branch)
begin
    PCsrc = (zeroFlagO & Branch) || (bneOO & !zeroFlagO);
end

DataMemory DM(readData,AluOut,WriteDataO,MemWrite,MemRead,clk);
MEM_WEB MW(MemoryWordReadO,AluResO,write_reg,MemToReg,regWrite,clk,readData,AluOut,RdOO,WBOO);

//wb block
assign write_data = (MemToReg==0)? MemoryWordReadO:AluResO;
endmodule


module instructionMemory(instruction, address);

input [6:0] address;
output reg [31:0] instruction;
reg[7:0] instruction_memory[511:0];

wire [6:0]wordAddress;
assign wordAddress = address;
                           //ð‘Žð‘›ð‘‘ $ð‘¡1, $ð‘¡2, $ð‘¡3 

initial
begin
//Arithmetic:

	//add $9, $15 , $20 (working)
	// instruction_memory[0] = 8'b000000_01;  
	// instruction_memory[1] = 8'b111_10100;     
	// instruction_memory[2] = 8'b01001000;
	// instruction_memory[3] = 8'b00_100000;

	//sub $9, $15 , $20 (working)
	// instruction_memory[0] = 8'b000000_01;  
	// instruction_memory[1] = 8'b111_10100;     
	// instruction_memory[2] = 8'b01001000;
	// instruction_memory[3] = 8'b00_100010;

	// addi $9, $0 ,9 (working)
	// instruction_memory[0] = 8'b00100000;  
	// instruction_memory[1] = 8'b00001001;     
	// instruction_memory[2] = 8'b00000000;
	// instruction_memory[3] = 8'b00001001;

//Load/Store:

	//lw $22, 1($20)  (working)
	// instruction_memory[0] = 8'b100011_10;
	// instruction_memory[1] = 8'b100_10110;
	// instruction_memory[2] = 8'b00000000;
	// instruction_memory[3] = 8'b00000001;

 	//sw $15, 0($0)
 	// instruction_memory[0] = 8'b101011_00;
 	// instruction_memory[1] = 8'b000_01111;
 	// instruction_memory[2] = 8'b00000000;
 	// instruction_memory[3] = 8'b00001100;

//Logic:

	//and $9, $15 , $20 (working)
	// instruction_memory[0] = 8'b000000_01;  
	// instruction_memory[1] = 8'b111_10100;     
	// instruction_memory[2] = 8'b01001000;
	// instruction_memory[3] = 8'b00_100100;

	//or $9, $15 , $20 (working)
	// instruction_memory[0] = 8'b000000_01;  
	// instruction_memory[1] = 8'b111_10100;     
	// instruction_memory[2] = 8'b01001000;
	// instruction_memory[3] = 8'b00_100101;

	//sll $9, $15 , 2 (working)
	// instruction_memory[0] = 8'b000000_00;
	// instruction_memory[1] = 8'b000_01111;  
	// instruction_memory[2] = 8'b01001_000;
	// instruction_memory[3] = 8'b10_000000;


	//srl $9, $15 , 1 (working)
	instruction_memory[0] = 8'b000000_00;
	instruction_memory[1] = 8'b000_01111;  
	instruction_memory[2] = 8'b01001_000;
	instruction_memory[3] = 8'b01_000010;

	// andi $10, $0, 10 (working)
	instruction_memory[4] = 8'b00110000;  
	instruction_memory[5] = 8'b00001010;     
	instruction_memory[6] = 8'b00000000;
	instruction_memory[7] = 8'b00001010;

	// ori $5 , $0, 7 (working)
	instruction_memory[8] = 8'b00110100;
	instruction_memory[9] = 8'b00000101;     
	instruction_memory[10] = 8'b00000000;
	instruction_memory[11] = 8'b00000111;

//Control Flow:

	//beq $20, $15 , 8 (working)
	// instruction_memory[0] = 8'b000100_10;
	// instruction_memory[1] = 8'b100_01111;
	// instruction_memory[2] = 8'b00000000;
	// instruction_memory[3] = 8'b00001000;

	//beq $15, $15 , 8 (working)
	// instruction_memory[0] = 8'b000100_01;
	// instruction_memory[1] = 8'b111_01111;
	// instruction_memory[2] = 8'b00000000;
	// instruction_memory[3] = 8'b00000100;

	//bne $20, $15 , 8 (working)
	// instruction_memory[0] = 8'b000101_10;
	// instruction_memory[1] = 8'b100_01111;
	// instruction_memory[2] = 8'b00000000;
	// instruction_memory[3] = 8'b00001000;

// Comparison:

	//slt $9, $15 , $20 (working)
	// instruction_memory[0] = 8'b000000_01;
	// instruction_memory[1] = 8'b111_10100;  
	// instruction_memory[2] = 8'b01001_000;
	// instruction_memory[3] = 8'b00_101010;

	//slt $9, $20 , $15 (working)
	// instruction_memory[0] = 8'b000000_10;
	// instruction_memory[1] = 8'b100_01111;  
	// instruction_memory[2] = 8'b01001_000;
	// instruction_memory[3] = 8'b00_101010;
end

always@(address)
begin
instruction = {instruction_memory[wordAddress], instruction_memory[wordAddress + 1],
					 instruction_memory[wordAddress + 2], instruction_memory[wordAddress + 3]};
end


endmodule


module IF_ID(InstructionO,PCO,Instruction,PC,clk);
output reg [31:0] InstructionO;
output reg [6:0] PCO;
input   [31:0] Instruction;
input [6:0] PC;
input clk ;

initial
begin
	InstructionO=32'b0;
	PCO= 7'b0;
end

always@(posedge clk)
    begin
      InstructionO = Instruction ;
      PCO = PC ;
    end

endmodule
 


module registerFile(read_reg_1, read_reg_2, write_reg, write_data, regWrite, read_data_1, read_data_2, clk);
input clk;
input [4:0] read_reg_1, read_reg_2, write_reg;
input [31:0] write_data;
input regWrite; // Control signal
output reg[31:0] read_data_1, read_data_2;
reg [5:0] i;
reg[31:0] registers[31:0]; //regfile making
initial                                                		//filling registerFile
begin
	registers[0] = 32'b0;
	registers[15] = 32'd5;
	registers[20] = 32'd3;
end

always @(read_reg_1,read_reg_2)																		//register read........ with check of zero register
begin
	read_data_1 = (read_reg_1==0)? 32'b0 : registers[read_reg_1];
	read_data_2 = (read_reg_2==0)? 32'b0 : registers[read_reg_2];	
end

always @(negedge clk)																				// writing in register........
begin
	if (regWrite) 
		begin
			registers[write_reg] = write_data;
			$display("-> %d %d ", write_reg, write_data);
		end	
end

initial                                                                                             // printing of all register at last clock
 begin
 	#191 begin
	 	for(i=0; i<32; i=i+1)
	 	begin
	 		$display("RegFile[%d]: %b \n",i,registers[i]);
	 	end
 	end
 end
endmodule

module controlUnit(instruction, EX, MEM, WB,Shamt,bne);
input [31:0] instruction;
output reg [4:0] EX;
output reg [2:0] MEM;
output reg [1:0] WB;
output reg [4:0] Shamt;
output reg bne;
reg [5:0] opcode;																					//variable defining	
reg ALUSrc=0, RegDst=0, Branch=0, MemRead=0, MemWrite=0, MemtoReg=0, RegWrite=0;					//braking EX,MEM,WB into parts
reg [2:0] ALUOp=3'b0;																				//ALUOP op for operation
initial																								//initial
begin
	opcode= 6'b0;
	EX= 5'b0;
	MEM= 3'b0;
	WB= 2'b0;
	Shamt= 5'b0;
	bne=0;
end
always@(instruction)
begin
	opcode = {instruction[31],instruction[30],instruction[29],instruction[28],instruction[27],instruction[26]};
	case (opcode)
	6'b000000:																						// for R
		begin
		ALUOp = 3'b010;
		RegDst = 1'b1;
		RegWrite = 1'b1;
		ALUSrc = 1'b0;
		MemWrite = 1'b0;
		MemRead = 1'b0;
		MemtoReg = 1'b1;
		Branch = 1'b0;
		bne=0;
	end	
	6'b001000: 																						// for ADDI
	begin
		ALUOp = 3'b011;
		RegDst = 1'b0; 
		RegWrite = 1'b1;
		ALUSrc = 1'b1; 
		MemWrite = 1'b0;
		MemRead = 1'b0;
		MemtoReg = 1'b1;
		Branch = 1'b0;
		bne=0;
	end	
	6'b001100: 																						// ANDI
	begin
		ALUOp = 3'b100;
		RegDst = 1'b0;// changed
		RegWrite = 1'b1;
		ALUSrc = 1'b1; // changed
		MemWrite = 1'b0;
		MemRead = 1'b0;
		MemtoReg = 1'b1;// changed
		Branch = 1'b0;
		bne=0;
	end	
	6'b001101:																						//ORI
	begin
		ALUOp = 3'b101;
		RegDst = 1'b0;// changed
		RegWrite = 1'b1;
		ALUSrc = 1'b1; //changed
		MemWrite = 1'b0;
		MemRead = 1'b0;
		MemtoReg = 1'b1;// changed
		Branch = 1'b0;
		bne=0;
	end
	6'b100011: 																						// LW
	begin
		ALUOp = 3'b000;
		RegDst = 1'b0;// changed
		RegWrite = 1'b1;
		ALUSrc = 1'b1; // changed
		MemWrite = 1'b0;
		MemRead = 1'b1;
		MemtoReg = 1'b0;// changed
		Branch = 1'b0;
		bne=0;
	end
	6'b101011: 																						// SW
	begin
		ALUOp = 3'b000;
		RegDst = 1'b0;
		RegWrite = 1'b0;
		ALUSrc = 1'b1;
		MemWrite = 1'b1;
		MemRead = 1'b0;
		MemtoReg = 1'b0;
		Branch = 1'b0;
		bne=0;
	end
	6'b000100: 																						// BEQ
	begin
		ALUOp = 3'b001;
		RegDst = 1'b0;
		RegWrite = 1'b0;
		ALUSrc = 1'b0;
		MemWrite = 1'b0;
		MemRead = 1'b0;
		MemtoReg = 1'b0;
		Branch = 1'b1;
		bne=0;
	end
		6'b000101: 																					//BNE
	begin
		ALUOp = 3'b001;
		RegDst = 1'b0;
		RegWrite = 1'b0;
		ALUSrc = 1'b0;
		MemWrite = 1'b0;
		MemRead = 1'b0;
		MemtoReg = 1'b0;
		Branch = 1'b0;
		bne=1;
	end
	endcase
	 WB =  {RegWrite,MemtoReg};
 	MEM =  {Branch,MemRead,MemWrite};
 	EX =  {RegDst,ALUOp,ALUSrc};
	 Shamt = {instruction[10],instruction[9],instruction[8],instruction[7],instruction[6]};
end
endmodule

module ID_EX(PCO,ReadReg1O,ReadReg2O,ShamtO,OffsetO,RtO,RdO,WBO,MEMO,ALUop,ALUSrc,RegDst,PC,bneO,clk,
    ReadReg1,ReadReg2,Shamt,Offset,Rt,Rd,WB,MEM,EX,bne);

output reg [31:0] ReadReg1O,ReadReg2O,OffsetO;  													//i/o defining
output reg [6:0] PCO;
output reg [4:0] RtO,RdO;
output reg [1:0] WBO;
output reg [2:0] MEMO;
output reg [2:0] ALUop;
output reg ALUSrc, RegDst;
output reg [4:0] ShamtO;
output reg bneO;
input clk ;
input [31:0] ReadReg1,ReadReg2,Offset;
input [6:0] PC;
input [4:0] Rt,Rd;
input [1:0] WB;
input [2:0]MEM;
input [4:0] EX;
input  [4:0] Shamt;
input bne;

initial																								// initial all zero
begin
    ReadReg1O=32'b0;
    ReadReg2O=32'b0;
    OffsetO=32'b0;
    PCO=7'b0;
    RtO=5'b0;
    RdO=5'b0;
    WBO=2'b0;
    MEMO=3'b0;
    ALUop=3'b0;
    ALUSrc=0;
    RegDst=0;
    ShamtO=5'b0;
    bneO=0;
end

always@(posedge clk)
    begin
        ReadReg1O <= ReadReg1;
        ReadReg2O <= ReadReg2;
        OffsetO <= Offset;
        PCO <= PC;
        RtO<= Rt;
        RdO<= Rd;
        WBO<= WB;
        MEMO<= MEM;
        ALUSrc<= EX[0];
        ALUop<= {EX[3],EX[2],EX[1]};
        RegDst<= EX[4];
        ShamtO<= Shamt;
        bneO<=bne;
    end
	
endmodule
 
module ALU(zeroFlag,out,reg1,reg2,ALUcontrol,shamt);
input [31:0] reg1,reg2;
input [2:0] ALUcontrol; 
input [4:0] shamt;
output reg [31:0] out;
output reg zeroFlag;
always @ (reg1, reg2, ALUcontrol)
begin
    case (ALUcontrol)
        0: out = reg1 + reg2;
        1: out = reg1 - reg2;
        2: out = reg1 & reg2;
        3: out = reg1 | reg2;
        4: out = reg2 << shamt;
        5: out = reg2 >> shamt;
        6: out = reg1<reg2? 32'b11111111111111111111111111111111:32'b0;
    endcase
end
 always @ (out)
begin
    zeroFlag = (out == 0)? 1:0;
end
endmodule
 
module ALUControl(ALUcontrol,ALUop,funct);
input[2:0] ALUop;
input[5:0] funct;
output reg [2:0] ALUcontrol;
always @(ALUop, funct)
begin
case(ALUop)
3'b000: ALUcontrol = 0; //lw,sw
3'b001: ALUcontrol = 1; //beq,bne
3'b011: ALUcontrol = 0; //addi
3'b100: ALUcontrol = 2; //andi
3'b101: ALUcontrol = 3; //ori
3'b010:  begin
            case(funct)
            6'b10_0000: ALUcontrol= 0; //add
            6'b10_0010: ALUcontrol= 1; //sub
            6'b10_0100: ALUcontrol= 2; //and
            6'b10_0101: ALUcontrol= 3; //or
            6'b00_0000: ALUcontrol= 4; //sll
            6'b00_0010: ALUcontrol= 5; //srl
            6'b10_1010: ALUcontrol= 6; //slt
            endcase
        end
endcase
end
endmodule

module EX_MEM(WriteDataO,AluResO,BranchAddressO,RdO,WBO,MemRead, MemWrite, Branch,ZFO,bneO,clk,
WriteData,AluRes,BranchAddress,Rd,WB,MEM,ZF,bne);
output reg [31:0] AluResO,WriteDataO;
output reg [6:0] BranchAddressO;
output reg [4:0] RdO;
output reg [1:0] WBO;
output reg ZFO, MemRead, MemWrite, Branch;
output reg bneO; 
input clk ;
input [31:0] AluRes,PC,WriteData;
input [6:0]BranchAddress;
input [4:0] Rd;
input [1:0] WB;
input [2:0] MEM;
input ZF ;
input bne;
initial
begin
    AluResO=32'b0;
    WriteDataO=32'b0;
    BranchAddressO=7'b0;
    RdO=5'b0;
    WBO=2'b0;
    MemRead=0;
    MemWrite=0;
    Branch=0;
    ZFO=0;
    bneO=0;
end
always@(posedge clk)
    begin
        WriteDataO <= WriteData;
        AluResO <= AluRes;
        BranchAddressO <= BranchAddress;
        RdO <= Rd;
        WBO<= WB;
        MemWrite<=MEM[0];
        MemRead<= MEM[1];
        Branch<= MEM[2];
        ZFO<= ZF;
        bneO<=bne;
    end
endmodule


module DataMemory(readData,wordAddress,writeData,MemWrite,MemRead,clk);
input MemWrite,MemRead;
input [31:0] wordAddress; // too large for memory check if it works 
input [31:0] writeData;
input clk ;
output reg [31:0]readData;
reg [7:0] memory[511:0];
reg [8:0] i;
initial
begin
	memory[4] = 8'h0f;
	memory[5] = 8'h0a;
	memory[6] = 8'h0f;
	memory[7] = 8'h07;
	memory[8] = 8'h00;
	memory[9] = 8'h11;
	memory[10] = 8'hff;
	memory[11] = 8'h01;
end
always@(negedge clk)
begin
		if(MemWrite)
		begin

		  	memory[wordAddress] = writeData[31:24];
		 	memory[wordAddress+1] = writeData[23:16];
		  	memory[wordAddress+2] = writeData[15:8];
		  	memory[wordAddress+3] = writeData[7:0];

		end
 end
always@(wordAddress)
begin
if(MemRead)
begin
 readData = {memory[wordAddress],memory[wordAddress+1],memory[wordAddress+2],memory[wordAddress+3]};
end
 end
 initial
 begin
 	#191 begin
	 	for(i=0; i<128; i=i+1)
	 	begin
	 		$display("Mem[%d]: %b \n",i,{memory[i*4],memory[i*4+1],memory[i*4+2],memory[i*4+3]});
	 	end
 	end
 end
endmodule

module MEM_WEB(MemoryWordReadO,AluResO,RdO,MemToReg,RegWrite,clk,
	MemoryWordRead,AluRes,Rd,WB);
output reg [31:0] MemoryWordReadO,AluResO;
output reg [4:0] RdO;
output reg RegWrite;
output reg MemToReg ;
input clk ;
input   [31:0] MemoryWordRead,AluRes;
input [4:0] Rd;
input [1:0] WB;

initial
begin
	MemoryWordReadO=32'b0;
	AluResO=32'b0;
	RdO=5'b0;
	RegWrite=0;
	MemToReg=0;
end
always@(posedge clk)
    begin
        MemoryWordReadO = MemoryWordRead;
        AluResO = AluRes;
        RdO = Rd;
        RegWrite = WB[1];
        MemToReg = WB[0];
    end

endmodule