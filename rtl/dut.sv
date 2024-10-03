//---------------------------------------------------------------------------
// DUT - Mini project 
//---------------------------------------------------------------------------
`include "common.vh"

module MyDesign(
//---------------------------------------------------------------------------
//System signals
  input wire reset_n                      ,  
  input wire clk                          ,

//---------------------------------------------------------------------------
//Control signals
  input wire dut_valid                    , 
  output wire dut_ready                   ,

//---------------------------------------------------------------------------
//input SRAM interface
  output wire                           dut__tb__sram_input_write_enable  ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_input_write_address ,
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_input_write_data    ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_input_read_address  , 
  input  wire [`SRAM_DATA_RANGE     ]   tb__dut__sram_input_read_data     ,     

//weight SRAM interface
  output wire                           dut__tb__sram_weight_write_enable  ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_weight_write_address ,
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_weight_write_data    ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_weight_read_address  , 
  input  wire [`SRAM_DATA_RANGE     ]   tb__dut__sram_weight_read_data     ,     

//result SRAM interface
  output wire                           dut__tb__sram_result_write_enable  ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_result_write_address ,
  output wire [`SRAM_DATA_RANGE     ]   dut__tb__sram_result_write_data    ,
  output wire [`SRAM_ADDR_RANGE     ]   dut__tb__sram_result_read_address  , 
  input  wire [`SRAM_DATA_RANGE     ]   tb__dut__sram_result_read_data          

);
/*----------------------start of my design----------------------*/

`define NUM_STATE 7
//mxn * nxp -> each iter has n ops, total mxp iters
//internal control signal
//FSM - one-hot encoding
logic [4:0] state,next_state;
//control: done this iter or total iter
logic read_iter_done,total_done;
//control: array size
logic get_matrix_dim,keep_matrix_dim;
//control: addr sel increment or keep or set to 0
enum_sel [1:0] read_addr_sel, write_addr_sel;
//control: accumulate the result of multiplication
logic accu_flg;
//control: write
logic write_first,write_en_i;
//control: signal control dut_ready
logic set_dut_ready;
//internal data container
logic [`SRAM_DATA_RANGE:0] read_data_input,read_data_weight,accum_result,mac_result_z;
//internal addr container
logic [`SRAM_ADDR_RANGE:0] read_addr_input,read_addr_weight,write_addr_result;
//internal write enable signal
logic write_en_reg;
//assume input_col = weight row, no error checking
logic [15:0] input_row,input_col,weight_col;
logic [15:0] read_counter,write_counter,iter_counter;

/*control*/
typedef enum logic { 
  IDLE               = `NUM_STATE'b000_0001,
  READ_DIM           = `NUM_STATE'b000_0010, //during IDLE, read addr is already 0, no need to wait one more cycle
  READ_FIRST_DATA    = `NUM_STATE'b000_0100,
  READ_AND_ACCU      = `NUM_STATE'b000_1000,
  WAIT_ACCU          = `NUM_STATE'b001_0000, //wait one cycle to finish accumulation
  WRITE_RESULT       = `NUM_STATE'b010_0000,
  COMPLETE           = `NUM_STATE'b100_0000,
 } enum_state;

typedef enum logic { 
  SET_TO_ZERO_0,
  SET_TO_ZERO_1, //dummy
  KEEP         ,
  INCREMENT    
} enum_sel;

//FSM
//async reset
always @(posedge clk or negedge reset_n) begin : fsm_state_transition
  if(!reset_n)begin
    state <= IDLE;
  end else begin
    state <= next_state;
  end
end

always @(*) begin : fsm_state_definition
  case(state) 
  IDLE:begin
    if(dut_valid)begin
      set_dut_ready   = 1'b0;
      read_addr_sel   = SET_TO_ZERO_0;
      write_addr_sel  = SET_TO_ZERO_0;
      accu_flg        = 1'b0;
      write_en_i      = 1'b0;
      get_matrix_dim  = 1'b0;
      keep_matrix_dim = 1'b0;
      next_state      = READ_DIM;
    end
    else begin
      set_dut_ready   = 1'b1;
      read_addr_sel   = SET_TO_ZERO_0;
      write_addr_sel  = SET_TO_ZERO_0;
      accu_flg        = 1'b0;
      write_en_i      = 1'b0;
      get_matrix_dim  = 1'b0;
      keep_matrix_dim = 1'b0;
      next_state      = IDLE;
    end
  end

  READ_DIM:begin
    set_dut_ready   = 1'b0;
    read_addr_sel   = INCREMENT;
    write_addr_sel  = SET_TO_ZERO_0;
    accu_flg        = 1'b0;
    write_en_i      = 1'b0;
    get_matrix_dim  = 1'b1;
    keep_matrix_dim = 1'b0;
    next_state      = READ_FIRST_DATA;
  end

  READ_FIRST_DATA:begin
    set_dut_ready   = 1'b0;
    read_addr_sel   = INCREMENT;
    write_addr_sel  = KEEP;
    accu_flg        = 1'b0;
    write_en_i      = 1'b0;
    get_matrix_dim  = 1'b0;
    keep_matrix_dim = 1'b1;
    next_state      = READ_AND_ACCU;
  end

  READ_AND_ACCU:begin
    if(read_iter_done)begin
      set_dut_ready   = 1'b0;
      read_addr_sel   = KEEP;
      write_addr_sel  = KEEP;
      accu_flg        = 1'b1;
      write_en_i      = 1'b0;
      get_matrix_dim  = 1'b0;
      keep_matrix_dim = 1'b1;
      next_state      = WAIT_ACCU;
    end else begin
      set_dut_ready   = 1'b0;
      read_addr_sel   = INCREMENT;
      write_addr_sel  = KEEP;
      accu_flg        = 1'b1;
      write_en_i      = 1'b0;
      get_matrix_dim  = 1'b0;
      keep_matrix_dim = 1'b1;
      next_state      = READ_AND_ACCU;
    end
  end

  WAIT_ACCU:begin
    set_dut_ready   = 1'b0;
    read_addr_sel   = KEEP;
    write_addr_sel  = KEEP;
    accu_flg        = 1'b1;
    write_en_i      = 1'b0;
    get_matrix_dim  = 1'b0;
    keep_matrix_dim = 1'b1;
    next_state      = WRITE_RESULT;
  end

  WRITE_RESULT:begin
    if(total_done)begin
      set_dut_ready   = 1'b0;
      read_addr_sel   = KEEP;
      write_addr_sel  = KEEP;
      accu_flg        = 1'b1;
      write_en_i      = 1'b1;
      get_matrix_dim  = 1'b0;
      keep_matrix_dim = 1'b1;
      next_state      = COMPLETE;
    end else begin
      set_dut_ready   = 1'b0;
      read_addr_sel   = KEEP;
      write_addr_sel  = write_first ? KEEP : INCREMENT;
      accu_flg        = 1'b1;
      write_en_i      = 1'b1;
      get_matrix_dim  = 1'b0;
      keep_matrix_dim = 1'b1;
      next_state      = READ_FIRST_DATA;
    end
  end

  COMPLETE:begin
    set_dut_ready   = 1'b1;
    read_addr_sel   = SET_TO_ZERO_0;
    write_addr_sel  = SET_TO_ZERO_0;
    accu_flg        = 0'b1;
    write_en_i      = 0'b0;
    get_matrix_dim  = 0'b0;
    keep_matrix_dim = 0'b1;
    next_state      = IDLE;
  end

  endcase
end

//dut_ready
assign dut_ready = set_dut_ready;

//read data logic
always @(posedge clk)begin
  if(!reset_n)begin
    read_data_input   <= 0;
    read_data_weight  <= 0;
  end else begin
    read_data_input   <= tb__dut__sram_input_read_data;
    read_data_weight  <= tb__dut__sram_weight_read_data;
  end
end

assign dut__tb__sram_input_read_address  = read_addr_input;
assign dut__tb__sram_weight_read_address = read_addr_weight;


//read addr logic
always @(posedge clk)begin
  if(!reset_n)begin
    read_addr_input   <= 0;
    read_addr_weight  <= 0;
    iter_counter      <= 0;
  end else begin
      if(read_addr_sel==SET_TO_ZERO_0 or read_addr_sel==SET_TO_ZERO_1)begin
        read_addr_input   <= 0;
        read_addr_weight  <= 0;
        iter_counter      <= 0;
      end
      else if(read_addr_sel == INCREMENT)begin
        read_addr_input    <=  read_addr_input + 1;
        read_addr_weight   <=  read_addr_weight + 1;
        if(read_addr_input < input_col*(iter_counter+1))begin
          iter_counter     <= iter_counter;
          read_iter_done   <= 0;
        end else begin
          iter_counter     <= iter_counter+1;
          read_iter_done   <= 1;
        end
        
      end
    //imply memory for KEEP
  end
end

assign dut__tb__sram_result_write_data = write_data;
//write data logic
always @(posedge clk)begin
  if(!reset_n)begin
    write_data   <= 0;
  end else begin
    write_data   <= write_en_i ? accum_result:0;
  end
end

assign dut__tb__sram_result_write_enable  = write_en_reg;
assign dut__tb__sram_result_write_address = write_addr_result;

//after first iteration, write_first become 0 so write addr start to increase
assign write_first = (write_counter == 0);
assign total_done  = (write_counter == input_row*weight_col-1);

//write addr logic
always @(posedge clk)begin
  if(!reset_n)begin
    write_addr_result   <= 0;
    write_counter       <= 0;
  end else begin
    if(write_addr_sel == SET_TO_ZERO_0 or write_addr_sel == SET_TO_ZERO_1)begin
      write_addr_result <= 0;
      write_counter       <= 0;
    end
    else if(write_addr_sel == INCREMENT) begin
      write_addr_result <= write_addr_result + 1;
      write_counter     <= write_counter + 1;
    end
  end
  //imply memory for KEEP
end

//write en logic
always @(posedge clk)begin
  if(!reset_n)begin
    write_en_reg <= 0;
  end else begin
    write_en_reg  <= write_en_i;
  end
end

//dim logic
always @(posedge clk)begin
  if(!reset_n)begin
    input_row   <= 0;
    input_col   <= 0;
    weight_col  <= 0;
  end else begin
    input_row  <= get_matrix_dim?tb__dut__sram_input_read_data[31:16]:(keep_matrix_dim?input_row:0);
    input_col  <= get_matrix_dim?tb__dut__sram_input_read_data[15:0]:(keep_matrix_dim?input_col:0);
    weight_col <= get_matrix_dim?tb__dut__sram_weight_read_data[15:0]:(keep_matrix_dim?weight_col:0);
  end
end


//accumulate logic
always @(posedge clk)begin
  if(!reset_n)begin
    accum_result <= 0;
  end else begin
    if(!total_done)begin
      accum_result <= accu_flg?mac_result_z:accum_result;
    end else begin
      accum_result <= 0;
    end
  end
end



/*----------------------end of my design----------------------*/

DW_fp_mac_inst 
  FP_MAC ( 
  .inst_a(tb__dut__sram_input_read_data),
  .inst_b(tb__dut__sram_weight_read_data),
  .inst_c(accum_result),
  .inst_rnd(inst_rnd),
  .z_inst(mac_result_z),
  .status_inst()
);

endmodule

module DW_fp_mac_inst #(
  parameter inst_sig_width = 23,
  parameter inst_exp_width = 8,
  parameter inst_ieee_compliance = 0 // These need to be fixed to decrease error
) ( 
  input wire [inst_sig_width+inst_exp_width : 0] inst_a,
  input wire [inst_sig_width+inst_exp_width : 0] inst_b,
  input wire [inst_sig_width+inst_exp_width : 0] inst_c,
  input wire [2 : 0] inst_rnd,
  output wire [inst_sig_width+inst_exp_width : 0] z_inst,
  output wire [7 : 0] status_inst
);

  // Instance of DW_fp_mac
  DW_fp_mac #(inst_sig_width, inst_exp_width, inst_ieee_compliance) U1 (
    .a(inst_a),
    .b(inst_b),
    .c(inst_c),
    .rnd(inst_rnd),
    .z(z_inst),
    .status(status_inst) 
  );
endmodule: DW_fp_mac_inst