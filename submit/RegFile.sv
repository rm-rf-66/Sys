`include "core_struct.vh"
module RegFile (
  input clk,
  input rst,
  input we,
  input CorePack::reg_ind_t  read_addr_1,
  input CorePack::reg_ind_t  read_addr_2,
  input CorePack::reg_ind_t  write_addr,
  input  CorePack::data_t write_data,
  output CorePack::data_t read_data_1,
  output CorePack::data_t read_data_2
);
  import CorePack::*;

  integer i;
  data_t register [1:31]; 

always @(posedge clk) begin
    if (rst) begin 
        for (i = 1; i < 32; i = i + 1)begin
            register[i] = 0;
        end
    end else if(we && (write_addr != 0)) begin 
        register[write_addr] <= write_data;
    end
end

assign read_data_1 = (read_addr_1 == 0) ? 0 : register[read_addr_1];
assign read_data_2 = (read_addr_2 == 0) ? 0 : register[read_addr_2];
endmodule
