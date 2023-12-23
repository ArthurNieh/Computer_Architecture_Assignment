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


// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // TODO: any declaration
        reg [BIT_W-1:0] PC, next_PC;
        wire mem_cen, mem_wen;
        wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
        wire mem_stall;

        reg [BIT_W-1:0] mem_data, mem_addr, mem_wdata, mem_rdata;

        // for Reg_file
        wire [BIT_W-1:0] rs1, rs2;

        // from Controller
        wire        branch, mem_read, mem_to_reg, mem_write, ALU_Src, reg_write, jal, jalr, auipc;
        wire [1:0]  ALU_Op;

        // for ALU
        wire        in_1, in_2;

        // ImmGen
        wire [31:0] imm;



// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: any wire assignment
    assign in_1 = rs1;
    assign in_2 = ALU_Src ? imm : rs2;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------

    // TODO: Reg_file wire connection
    Reg_file reg_0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (reg_write),         
        .rs1    (i_IMEM_data[19:15]),                
        .rs2    (i_IMEM_data[24:20]),                
        .rd     (i_IMEM_data[11:7]),                 
        .wdata  (o_DMEM_wdata),             
        .rdata1 (rs1),           
        .rdata2 (rs2)
    );

    Controller control_0(
        .opcode (i_IMEM_data[6:0]),
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

    ImmGen immgen_0(
        .instr(i_IMEM_data[31:0]),
        .ALUOp(ALU_Op),
        .imm_32(imm)
    );

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    
    // Todo: any combinational/sequential circuit

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
        end
        else begin
            PC <= next_PC;
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

module MUL_unit(
    i_clk, i_rst_n, 
    i_start, 
    i_opperand_a, 
    i_opperand_b, 
    counter, 
    o_result, 
    o_valid
    );
    // Todo: HW2
    parameter IDLE = 1'b0;
    parameter MUL = 1'b1;

    input i_clk, i_rst_n, i_start;
    input [31:0] i_opperand_a, i_opperand_b;
    input [4:0] counter, counter_nxt;

    output [63:0] o_result;
    output o_valid;

    reg [63:0] ans, ans_nxt, o_data;
    reg [4:0] counter_nxt;
    reg done;

    assign o_result = o_data;

    always@(*) begin
        case (counter)
            0: begin
                ans_nxt = i_opperand_a;
            end
            1: begin
                ans_nxt = ans_nxt * i_opperand_b;
            end
            default: begin
                ans_nxt = ans_nxt;
            end
        endcase
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            ans <= 0;
            counter <= 0;
            done <= 0;
        end
        else begin
            ans <= ans_nxt;
            counter <= counter_nxt;
            done <= done;   
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

module Controller(opcode, Branch, MemRead, MemtoReg, ALUOp, MemWrite, ALUSrc, RegWrite, Jal, Jalr, Auipc);
    input   [6:0]   opcode;
    output          Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, Jal, Jalr, Auipc;
    output  [1:0]   ALUOp;

    reg     [10:0]  total_out;
    assign  {ALUSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, Jal, Jalr, ALUOp, Auipc} = total_out;
        //   10         9          8        7       6          5     4    3     2 1     0
    always @(*) begin
        total_out = 11'b00000000000;
        case(opcode)
            7'b0110011: begin   // R-type
                total_out[2] = 1; // ALUOp = 10
                total_out[8] = 1; // RegWrite
            end
            7'b0010011: begin   // I-type
                total_out[2] = 1; // ALUOp = 10
                total_out[8] = 1; // RegWrite
                total_out[10] = 1;// ALUSrc
            end
            7'b0000011: begin   // lw
                total_out[7] = 1; // MemRead
                total_out[8] = 1; // RegWrite
                total_out[9] = 1; // MemtoReg
                total_out[10] = 1;// ALUSrc
            end
            7'b0100011: begin   //sw
                total_out[6] = 1; // MemRead
                total_out[10] = 1;// ALUSrc
            end
            7'b1100011: begin   // beq, bne, blt, bge
                total_out[1] = 1; // ALUOp = 01
                total_out[5] = 1; // Branch
            end
            7'b1101111: begin   // jal
                total_out[4] = 1; // Jal
                total_out[8] = 1; // RegWrite
                total_out[10] = 1;// ALUSrc
            end
            7'b1100111: begin   // jalr
                total_out[3] = 1; // Jalr
                total_out[8] = 1; // RegWrite
                total_out[10] = 1;// ALUSrc
            end
            7'b0010111: begin   // auipc
                total_out[0] = 1; // Auipc
                total_out[8] = 1; // RegWrite
                total_out[10] = 1;// ALUSrc
            end
            7'b0 // mul
            7'b1110011:  // ecall 
        endcase
    end    

endmodule

module ImmGen(instr, ALUOp, imm_32);
    input   [31:0] instr;
    input   [ 1:0] ALUOp;
    output reg [31:0] imm_32;  // output reg??

    always @(*) begin
        case(ALUOp)
            // I-type
            2'b10:  imm_32 = {{20{instr[31]}}, instr[31:20]};
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