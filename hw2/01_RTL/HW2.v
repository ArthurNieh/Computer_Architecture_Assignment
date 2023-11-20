module ALU #(
    parameter DATA_W = 32
)
(
    input                       i_clk,   // clock
    input                       i_rst_n, // reset

    input                       i_valid, // input valid signal
    input [DATA_W - 1 : 0]      i_A,     // input operand A
    input [DATA_W - 1 : 0]      i_B,     // input operand B
    input [         2 : 0]      i_inst,  // instruction

    output [2*DATA_W - 1 : 0]   o_data,  // output value
    output                      o_done   // output valid signal
);
// Do not Modify the above part !!!

// Parameters
    // ======== choose your FSM style ==========
    // 1. FSM based on operation cycles
    parameter S_IDLE           = 2'd0;
    parameter S_ONE_CYCLE_OP   = 2'd1;
    parameter S_MULTI_CYCLE_OP = 2'd2;
    // 2. FSM based on operation modes
    // parameter S_IDLE = 4'd0;
    // parameter S_ADD  = 4'd1;
    // parameter S_SUB  = 4'd2;
    // parameter S_AND  = 4'd3;
    // parameter S_OR   = 4'd4;
    // parameter S_SLT  = 4'd5;
    // parameter S_SRA  = 4'd6;
    // parameter S_MUL  = 4'd7;
    // parameter S_DIV  = 4'd8;
    // parameter S_OUT  = 4'd9;

// Wires & Regs
    // Todo
    // state
    reg  [         1: 0] state, state_nxt; // remember to expand the bit width if you want to add more states!
    // load input
    reg  [  DATA_W-1: 0] operand_a, operand_a_nxt;
    reg  [  DATA_W-1: 0] operand_b, operand_b_nxt;
    reg  [         2: 0] inst, inst_nxt;

    wire [  DATA_W-1: 0] add_buf, sub_buf;
    reg  [  2*DATA_W: 0] shift_reg, shift_reg_nxt;
    reg                  done, done_nxt;
    // reg                  valid;
    reg  [         4: 0] counter;

    
// Wire Assignments
    // Todo 
    assign add_buf = operand_a + operand_b;
    assign sub_buf = operand_a - operand_b;
    
    assign o_data = shift_reg[ 2*DATA_W-1: 0];
    assign o_done = done;
    
// Always Combination
    // load input
    always @(*) begin
        if (i_valid) begin
            operand_a_nxt = i_A;
            operand_b_nxt = i_B;
            inst_nxt      = i_inst;
            // valid         = i_valid;
        end
        else begin
            operand_a_nxt = operand_a;
            operand_b_nxt = operand_b;
            inst_nxt      = inst;
            // valid         = i_valid;
        end
    end
    // Todo: FSM
    always @(*) begin
        case(state)
            S_IDLE           :  begin
                if (i_valid) state_nxt = (i_inst >= 6 ) ? S_MULTI_CYCLE_OP : S_ONE_CYCLE_OP;
                else state_nxt = state;
            end
            S_ONE_CYCLE_OP   :  state_nxt = S_IDLE;
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
                done_nxt = 1'b0;
            end
            S_ONE_CYCLE_OP  : begin
                case (inst)
                    3'd0    : begin
                        if ((operand_a[DATA_W-1] == 1) && (operand_b[DATA_W-1] == 1) && (add_buf[DATA_W-1] == 0))
                            shift_reg_nxt[31:0] = {1'b1, 31'b0};
                        else if ((operand_a[DATA_W-1] == 0) && (operand_b[DATA_W-1] == 0) && (add_buf[DATA_W-1] == 1))
                            shift_reg_nxt[31:0] = {1'b0, {31{1'b1}}};
                        else
                            shift_reg_nxt[31:0] = add_buf;
                    end
                    3'd1    : begin
                        if ((operand_a[DATA_W-1] == 1) && (operand_b[DATA_W-1] == 0) && (sub_buf[DATA_W-1] == 0))
                            shift_reg_nxt[31:0] = {1'b1, 31'b0};
                        else if ((operand_a[DATA_W-1] == 0) && (operand_b[DATA_W-1] == 1) && (sub_buf[DATA_W-1] == 1))
                            shift_reg_nxt[31:0] = {1'b0, {31{1'b1}}};
                        else
                            shift_reg_nxt[31:0] = sub_buf;
                    end
                    3'd2    : shift_reg_nxt[31:0] = operand_a & operand_b;
                    3'd3    : shift_reg_nxt[31:0] = operand_a | operand_b;
                    3'd4    : shift_reg_nxt[31:0] = ($signed(operand_a) < $signed(operand_b)) ? 32'd1 : 32'd0;
                    3'd5    : shift_reg_nxt[31:0] = $signed(operand_a) >>> $signed(operand_b);
                    default : shift_reg_nxt[31:0] = shift_reg[31:0];
                endcase
                shift_reg_nxt[64:32] = 33'b0;
                done_nxt = 1'b1;
            end
            S_MULTI_CYCLE_OP: begin
                case (inst)
                    3'd6    : begin
                        if (counter == 0)
                            shift_reg_nxt[31:0] = operand_a;
                        else
                            shift_reg_nxt[31:0] = shift_reg[31:0];
                        
                        if(shift_reg_nxt[0] == 1)
                            shift_reg_nxt[64:32] = shift_reg[63:32] + operand_b;
                        else
                            shift_reg_nxt[64:32] = shift_reg[63:32];
                        
                        shift_reg_nxt = shift_reg_nxt >> 1;
                        
                        if (counter == 31) 
                            done_nxt = 1'b1;
                        else 
                            done_nxt = 1'b0;
                    end
                    3'd7    :begin
                        if (counter == 0) begin
                            if ((operand_b == 1) && (operand_a[31] == 1)) begin
                                shift_reg_nxt[31:1] = operand_a[30:0];
                                shift_reg_nxt[32] = 0;
                                shift_reg_nxt[0] = 1;
                            end
                            else begin
                                shift_reg_nxt[32:1] = operand_a;
                                shift_reg_nxt[0] = 0;
                            end
                            shift_reg_nxt[63:33] = shift_reg[63:33];
                        end
                        else begin
                            // shift_reg_nxt[32:1] = shift_reg[32:1];
                            if (shift_reg[63:32] >= operand_b) begin
                                shift_reg_nxt[63:32] = shift_reg[63:32] - operand_b;
                                shift_reg_nxt[0] = 1;
                            end
                            else begin
                                shift_reg_nxt[63:32] = shift_reg[63:32];
                                shift_reg_nxt[0] = 0;
                            end
                            shift_reg_nxt[31:1] = shift_reg[31:1];
                        end
                        
                        
                        
                        if (counter == 31) 
                            done_nxt = 1'b1;
                        else begin
                            shift_reg_nxt = shift_reg_nxt << 1;
                            done_nxt = 1'b0;
                        end
                    end
                    default :begin
                        shift_reg_nxt = shift_reg;
                        done_nxt = done;
                    end
                endcase
            end
            default : begin
                shift_reg_nxt = shift_reg;
                done_nxt = 1'b0;
            end
        endcase
    end
    // Todo: output valid signal
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin 
            done <= 0;
        end
        else begin
            done <= done_nxt;
        end
    end

    // Todo: Sequential always block
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state       <= S_IDLE;
            operand_a   <= 0;
            operand_b   <= 0;
            inst        <= 0;
            shift_reg   <= 0;
        end
        else begin
            state       <= state_nxt;
            operand_a   <= operand_a_nxt;
            operand_b   <= operand_b_nxt;
            inst        <= inst_nxt;
            shift_reg   <= shift_reg_nxt;
        end
    end

endmodule