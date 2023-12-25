//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//
module CHIP #(                                                                                  //
    parameter BIT_W = 32                                                                        //
)(                                                                                              //
    // clock                                                                                    //
        input               i_clk,                                                              //
        input               i_rst_n,                                                            //
    // instruction memory                                                                       //
        input  [BIT_W-1:0]  i_IMEM_data,                                                        //
        output [BIT_W-1:0]  o_IMEM_addr,                                                        //
        output              o_IMEM_cen,                                                         //
    // data memory                                                                              //
        input               i_DMEM_stall,                                                       //
        input  [BIT_W-1:0]  i_DMEM_rdata,                                                       //
        output              o_DMEM_cen,                                                         //
        output              o_DMEM_wen,                                                         //
        output [BIT_W-1:0]  o_DMEM_addr,                                                        //
        output [BIT_W-1:0]  o_DMEM_wdata,                                                       //
    // finnish procedure                                                                        //
        output              o_finish,                                                           //
    // cache                                                                                    //
        input               i_cache_finish,                                                     //
        output              o_proc_finish                                                       //
);                                                                                              //
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any declaration
    parameter S_IDLE           = 2'd0;
    parameter S_ONE_CYCLE_OP   = 2'd1;
    parameter S_STALL          = 2'd2;
    parameter S_MULTI_CYCLE_OP = 2'd3;  // mul

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg [BIT_W-1:0] PC, PC_next;
        wire jump;
        // reg done_reg, done_nxt_reg;
        // wire done_wire;
        
        // wire mem_cen, mem_wen;
        // wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
        // wire mem_stall;
    // FSM
        reg [1:0] state, state_nxt;

    // for i/o control
        reg dcen,icen;

    // for Reg_file
        wire [BIT_W-1:0] rs1, rs2;
        wire [BIT_W-1:0] m_to_r, data_to_reg;

        wire done_reg_write;

    // from Controller
        wire        branch, mem_read, mem_to_reg, mem_write, ALU_Src, reg_write, jal, jalr, auipc;
        wire [1:0]  ALU_Op;

    // for ALU
        wire [      2:0] alu_control;
        wire             alu_done, is_mul;
        wire [BIT_W-1:0] alu_in_1, alu_in_2;
        wire [BIT_W-1:0] alu_out;

        wire [      4:0] mul_counter;
        wire [BIT_W-1:0] mul_product;

    // ImmGen
        wire [31:0] imm;



// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment
    // assign o_finish = done_reg;
// for instructoin i/o control
    assign o_IMEM_addr = PC;
    assign o_IMEM_cen = icen;
    
// for memory i/o control
    assign o_DMEM_cen = dcen;
    assign o_DMEM_wen = mem_write;
    assign o_DMEM_addr = alu_out;
    assign o_DMEM_wdata = rs2;
// alu
    assign alu_in_1 = auipc ? PC : rs1;
    assign alu_in_2 = ALU_Src ? imm : rs2;
// reg
    assign m_to_r = mem_to_reg ? i_DMEM_rdata : alu_out;
    assign data_to_reg = (jal|jalr) ? (PC+32'd4) : m_to_r; 
    assign done_reg_write = reg_write & alu_done;

    assign jump = (i_IMEM_data[14:12] == 3'b000 && branch && alu_out == 32'b0) ? 1 : // beq
                  (i_IMEM_data[14:12] == 3'b001 && branch && alu_out != 32'b0) ? 1 : // bne
                  (i_IMEM_data[14:12] == 3'b100 && branch && alu_out[31] == 1) ? 1 : // blt
                  (i_IMEM_data[14:12] == 3'b101 && branch && alu_out[31] == 0) ? 1 : 0; // bge

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg_0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (done_reg_write),         
        .rs1    (i_IMEM_data[19:15]),                
        .rs2    (i_IMEM_data[24:20]),                
        .rd     (i_IMEM_data[11:7]),                 
        .wdata  (data_to_reg),             
        .rdata1 (rs1),           
        .rdata2 (rs2)
    );

    Controller control_0(
        .opcode (i_IMEM_data[6:0]),
        .Finish (o_finish),
        .Branch (branch),
        .MemRead(mem_read),
        .MemtoReg(mem_to_reg),
        .ALUOp  (ALU_Op),
        .MemWrite(mem_write),
        .ALUSrc (ALU_Src),
        .RegWrite(reg_write),
        .Jal    (jal),
        .Jalr   (jalr),
        .Auipc  (auipc)
    );

    ALUControl alu_Control0(
        .ALUOp(ALU_Op),
        .funct7_5(i_IMEM_data[30]),
        .funct7_0(i_IMEM_data[25]),
        .funct3(i_IMEM_data[14:12]),
        .opcode(i_IMEM_data[6:0]),
        .ALU_Control(alu_control),
        .Is_Mul(is_mul)
    );

    ImmGen immgen_0(
        .instr(i_IMEM_data[31:0]),
        .ALUOp(ALU_Op),
        .imm_32(imm)
    );

    ALU alu_0(
        .i_A(alu_in_1),
        .i_B(alu_in_2),
        .ALU_Control(alu_control),
        .Counter(mul_counter),
        .Mul_Product(mul_product),
        .ALU_Out(alu_out),
        .Done(alu_done)
    );

    MUL mul_unit(
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_valid(is_mul),
        .i_A(alu_in_1),
        .i_B(alu_in_2),
        .o_data(mul_product),
        .Counter(mul_counter)
    );

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // Todo: any combinational/sequential circuit
// FSM
    always @(*) begin
        case(state)
            S_IDLE           : begin
                icen = 1;
                dcen = 0;
                if(mem_write|mem_read) state_nxt = S_ONE_CYCLE_OP;
                else if(is_mul) state_nxt = S_MULTI_CYCLE_OP;
                else state_nxt = S_IDLE;
            end
            S_ONE_CYCLE_OP   : begin
                icen = 0;
                dcen = 1;
                if(i_DMEM_stall) state_nxt = S_ONE_CYCLE_OP;
                else state_nxt = S_IDLE;
            end
            S_MULTI_CYCLE_OP : begin
                icen = 0;
                dcen = 0;
                if(alu_done) state_nxt = S_IDLE;
                else state_nxt = S_MULTI_CYCLE_OP;
            end
            default          : begin
                icen = 0;
                dcen = 0;
                state_nxt = S_IDLE;
            end
        endcase
    end

// PC control
    always @(*) begin
        if(o_IMEM_cen) begin
            if(jalr)            PC_next = $signed(rs1) + $signed(imm);
            else if(jal || jump)PC_next = PC + $signed(imm);
            else                PC_next = PC + 4;
        end
        else PC_next = PC;
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
            state <= S_IDLE;
        end
        else begin
            PC <= PC_next;
            state <= state_nxt;
        end
    end
    
endmodule

module Reg_file(i_clk, i_rst_n, wen, rs1, rs2, rd, wdata, rdata1, rdata2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input i_clk, i_rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] wdata;
    input [addr_width-1:0] rs1, rs2, rd;

    output [BITS-1:0] rdata1, rdata2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign rdata1 = mem[rs1];
    assign rdata2 = mem[rs2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (rd == i)) ? wdata : mem[i];
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module MUL(i_clk, i_rst_n, i_valid, i_A, i_B, o_data, Counter);

    input         i_clk;   // clock
    input         i_rst_n; // reset

    input         i_valid; // input valid signal
    input  [31:0] i_A;     // input operand A
    input  [31:0] i_B;     // input operand B

    output [31:0] o_data;  // output value
    output [ 4:0] Counter;

// Parameters
    parameter S_IDLE           = 1'b0;
    parameter S_MULTI_CYCLE_OP = 1'b1;
    parameter DATA_W = 32;
// Wires & Regs
    // state
    reg state, state_nxt;
    // load input
    reg  [  DATA_W-1: 0] operand_a, operand_a_nxt;
    reg  [  DATA_W-1: 0] operand_b, operand_b_nxt;

    wire [  DATA_W-1: 0] add_buf, sub_buf;
    reg  [  2*DATA_W: 0] shift_reg, shift_reg_nxt;
    reg  [         4: 0] counter;
// Wire Assignments
    assign add_buf = operand_a + operand_b;
    assign sub_buf = operand_a - operand_b;
    
    assign o_data = shift_reg_nxt[ DATA_W-1: 0];
    assign Counter = counter;
    
// Always Combination
    // load input
    always @(*) begin
        if (i_valid) begin
            operand_a_nxt = i_A;
            operand_b_nxt = i_B;
        end
        else begin
            operand_a_nxt = operand_a;
            operand_b_nxt = operand_b;
        end
    end
    // Todo: FSM
    always @(*) begin
        case(state)
            S_IDLE           :  begin
                if (i_valid) state_nxt = S_MULTI_CYCLE_OP;
                else state_nxt = state;
            end
            S_MULTI_CYCLE_OP :  state_nxt = (counter == 31) ? S_IDLE : state;           
            default : state_nxt = state;
        endcase
    end
    // Todo: Counter
    always @(posedge i_clk) begin 
        // S_MUL or S_DIV
        counter = (state == S_MULTI_CYCLE_OP) ? counter + 5'd1 : 5'd0;
    end

    // Todo: ALU output
    always @(*) begin
        case (state)
            S_IDLE          : begin
                shift_reg_nxt = 65'b0;
            end
            S_MULTI_CYCLE_OP: begin
                if (counter == 0)
                    shift_reg_nxt[31:0] = operand_a;
                else
                    shift_reg_nxt[31:0] = shift_reg[31:0];
                
                if(shift_reg_nxt[0] == 1)
                    shift_reg_nxt[64:32] = shift_reg[63:32] + operand_b;
                else
                    shift_reg_nxt[64:32] = shift_reg[64:32];
                
                shift_reg_nxt = shift_reg_nxt >> 1;
            end
            default : begin
                shift_reg_nxt = shift_reg;
            end
        endcase
    end

    // Todo: Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state       <= S_IDLE;
            operand_a   <= 0;
            operand_b   <= 0;
            shift_reg   <= 0;
        end
        else begin
            state       <= state_nxt;
            operand_a   <= operand_a_nxt;
            operand_b   <= operand_b_nxt;
            shift_reg   <= shift_reg_nxt;
        end
    end

endmodule

module Cache#(
        parameter BIT_W = 32,
        parameter ADDR_W = 32
    )(
        input i_clk,
        input i_rst_n,
        // processor interface
            input i_proc_cen,
            input i_proc_wen,
            input [ADDR_W-1:0] i_proc_addr,
            input [BIT_W-1:0]  i_proc_wdata,
            output [BIT_W-1:0] o_proc_rdata,
            output o_proc_stall,
            input i_proc_finish,
            output o_cache_finish,
        // memory interface
            output o_mem_cen,
            output o_mem_wen,
            output [ADDR_W-1:0] o_mem_addr,
            output [BIT_W*4-1:0]  o_mem_wdata,
            input [BIT_W*4-1:0] i_mem_rdata,
            input i_mem_stall,
            output o_cache_available,
        // others
        input  [ADDR_W-1: 0] i_offset
    );

    assign o_cache_available = 0; // change this value to 1 if the cache is implemented

    //------------------------------------------//
    //          default connection              //
    assign o_mem_cen = i_proc_cen;              //
    assign o_mem_wen = i_proc_wen;              //
    assign o_mem_addr = i_proc_addr;            //
    assign o_mem_wdata = i_proc_wdata;          //
    assign o_proc_rdata = i_mem_rdata[0+:BIT_W];//
    assign o_proc_stall = i_mem_stall;          //
    //------------------------------------------//

    // Todo: BONUS
endmodule

module Controller(opcode, Finish, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite, Jal, Jalr, Auipc);
    input   [6:0]   opcode;
    output          Finish, Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, Jal, Jalr, Auipc;
    output  [1:0]   ALUOp;

    reg     [11:0]  total_out;
    assign  {Finish, ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, Jal, Jalr, ALUOp, Auipc} = total_out;
        //     11     10         9          8        7       6          5     4    3     2 1     0
    
    always @(*) begin
        case(opcode)
            7'b0110011: total_out = 12'b000100000100; // R-type
            7'b0010011: total_out = 12'b010100000110; // I-type
            7'b0000011: total_out = 12'b011110000000; // lw
            7'b0100011: total_out = 12'b010001000000; // sw
            7'b1100011: total_out = 12'b000000100010; // beq, bne, blt, bge
            7'b1101111: total_out = 12'b010100010000; // jal
            7'b1100111: total_out = 12'b010100001000; // jalr
            7'b0010111: total_out = 12'b010100000001; // auipc
            7'b1110011: total_out = 12'b100000000000; // ecall
            default   : total_out = 12'b000000000000;
        endcase
    end
    
    // always @(*) begin
    //     total_out = 12'b000000000000;
    //     case(opcode)
    //         7'b0110011: begin   // R-type
    //             total_out[2] = 1; // ALUOp = 10
    //             total_out[8] = 1; // RegWrite
    //         end
    //         7'b0010011: begin   // I-type
    //             total_out[1] = 1;
    //             total_out[2] = 1; // ALUOp = 11
    //             total_out[8] = 1; // RegWrite
    //             total_out[10] = 1;// ALUSrc
    //         end
    //         7'b0000011: begin   // lw
    //             total_out[7] = 1; // MemRead
    //             total_out[8] = 1; // RegWrite
    //             total_out[9] = 1; // MemtoReg
    //             total_out[10] = 1;// ALUSrc
    //         end
    //         7'b0100011: begin   //sw
    //             total_out[6] = 1; // MemWrite
    //             total_out[10] = 1;// ALUSrc
    //         end
    //         7'b1100011: begin   // beq, bne, blt, bge
    //             total_out[1] = 1; // ALUOp = 01
    //             total_out[5] = 1; // Branch
    //         end
    //         7'b1101111: begin   // jal
    //             total_out[4] = 1; // Jal
    //             total_out[8] = 1; // RegWrite
    //             total_out[10] = 1;// ALUSrc
    //         end
    //         7'b1100111: begin   // jalr
    //             total_out[3] = 1; // Jalr
    //             total_out[8] = 1; // RegWrite
    //             total_out[10] = 1;// ALUSrc
    //         end
    //         7'b0010111: begin   // auipc
    //             total_out[0] = 1; // Auipc
    //             total_out[8] = 1; // RegWrite
    //             total_out[10] = 1;// ALUSrc
    //         end
    //         // 7'b0 // mul
    //         7'b1110011: begin  // ecall 
    //             total_out[11] = 1; // Finish
    //         end
    //     endcase
    // end
endmodule

module ImmGen(instr, ALUOp, imm_32);
    input       [31:0] instr;
    input       [ 1:0] ALUOp;
    output reg  [31:0] imm_32;  // output reg??

    always @(*) begin
        case(ALUOp)
            // I-type
            2'b11:  imm_32 = {{20{instr[31]}}, instr[31:20]};
            // beq, bne
            2'b01:  imm_32 = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};
            2'b00: begin
                case(instr[6:0])
                    7'b0000011: imm_32 = {{20{instr[31]}}, instr[31:20]}; // lw I-type
                    7'b0100011: imm_32 = {{20{instr[31]}}, instr[31:25], instr[11:7]}; // sw S-type
                    7'b1101111: imm_32 = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; // jal UJ-type
                    7'b1100111: imm_32 = {{20{instr[31]}}, instr[31:20]}; // jalr I-type
                    7'b0010111: imm_32 = {instr[31:12],12'b0}; // auipc U-type
                    default: imm_32 = 32'b0;
                endcase
            end
            default: imm_32 = 32'b0;
        endcase
    end
endmodule

module ALUControl(ALUOp, funct7_5, funct7_0, funct3, opcode, ALU_Control, Is_Mul);
    input      [1:0] ALUOp;
    input            funct7_5, funct7_0;
    input      [2:0] funct3;
    input      [6:0] opcode;
    output reg [2:0] ALU_Control;
    output reg       Is_Mul;

    always @(*) begin
        Is_Mul = 0;
        case(ALUOp)
            2'b11: begin // I-type
                case(funct3)
                    3'b000: ALU_Control = 3'b000;   // add (addi)
                    3'b001: ALU_Control = 3'b010;   // slli
                    3'b010: ALU_Control = 3'b011;   // slti
                    3'b101: ALU_Control = 3'b100;   // srai
                    default:ALU_Control = 3'b000;
                endcase
            end
            2'b10: begin // R-type
                case(funct3)
                    3'b000: begin
                        if(funct7_0) begin
                            ALU_Control = 3'b111;  // mul
                            Is_Mul = 1;
                        end
                        else if(funct7_5)   ALU_Control = 3'b001;  // sub
                        else                ALU_Control = 3'b000;  // add
                    end
                    3'b100: ALU_Control = 3'b101;   // xor
                    3'b111: ALU_Control = 3'b110;   // and
                    default:ALU_Control = 3'b000;
                endcase
            end
            2'b01:  ALU_Control = 3'b001; // sub (beq) (don't need)
            2'b00:  ALU_Control = 3'b000; // add (lw sw auipc jal jalr)
            default:ALU_Control = 3'b000;
        endcase
    end
endmodule

module ALU (i_A, i_B, ALU_Control, Counter, Mul_Product, ALU_Out, Done);
    input   [31:0] i_A, i_B;
    input   [ 2:0] ALU_Control;
    input   [ 4:0] Counter;
    input   [31:0] Mul_Product;
    output  [31:0] ALU_Out;
    output         Done;

    reg [31:0]  o_result;
    reg         o_finish;
    assign  ALU_Out = o_result;
    assign  Done = o_finish;

    always @(*) begin
        o_finish = 1'b1;
        case(ALU_Control)
            3'b000: o_result = $signed(i_A) + $signed(i_B);  // add
            3'b001: o_result = $signed(i_A) - $signed(i_B);  // sub
            3'b010: o_result = i_A << $signed(i_B);          // slli
            3'b011: o_result = ($signed(i_A) < $signed(i_B)) ? 32'b1 : 32'b0; // slti
            3'b100: o_result = i_A >>> $signed(i_B);         // srai
            3'b101: o_result = i_A ^ i_B;                    // xor
            3'b110: o_result = i_A & i_B;                    // and
            3'b111: begin                                    // mul
                o_result = Mul_Product;
                if(Counter == 31) o_finish = 1'b1;
                else              o_finish = 1'b0;
            end
        endcase
    end
endmodule
